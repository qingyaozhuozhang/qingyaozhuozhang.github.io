"""MkDocs hook: auto-build the homepage's recent-updates list.

It also hides page-level maintenance metadata such as "修改时间" and "参与者"
from rendered Markdown, so future notes do not need manual cleanup.
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import urllib.request
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, Tuple

AUTO_BLOCK = re.compile(
    r"<!-- AUTO_RECENT_UPDATES_START -->.*?<!-- AUTO_RECENT_UPDATES_END -->",
    re.S,
)

# Only strip maintenance metadata when it occupies its own line.
# This will not delete normal prose such as a Linux field description containing “修改时间”.
MAINTENANCE_LINE = re.compile(
    r"(?mi)^[ \t]*(?:[-*][ \t]+)?(?:\*\*)?"
    r"(?:修改时间|更新时间|创建时间|参与者|参与人员|编辑者|维护者)"
    r"\s*[:：].*?(?:\*\*)?[ \t]*(?:\n|$)"
)

_FRONT_MATTER = re.compile(r"\A---\s*\n(.*?)\n---\s*\n", re.S)
_H1 = re.compile(r"(?m)^#\s+(.+?)\s*$")

_CACHE: str | None = None


def _front_matter_value(text: str, key: str) -> str:
    m = _FRONT_MATTER.search(text)
    if not m:
        return ""
    body = m.group(1)
    km = re.search(rf"(?m)^{re.escape(key)}\s*:\s*(.+?)\s*$", body)
    if not km:
        return ""
    return km.group(1).strip().strip('"\'')


def _plain_summary(text: str, limit: int = 90) -> str:
    description = _front_matter_value(text, "description")
    if description:
        return description[:limit].rstrip("，,。；; ") + ("…" if len(description) > limit else "")

    body = _FRONT_MATTER.sub("", text, count=1)
    in_code = False
    for raw in body.splitlines():
        line = raw.strip()
        if line.startswith("```"):
            in_code = not in_code
            continue
        if in_code or not line:
            continue
        if line.startswith(("#", "<!--", "!!!", "???", "|", "- ", "* ", ">")):
            continue
        line = re.sub(r"!?\[([^\]]+)\]\([^\)]+\)", r"\1", line)
        line = re.sub(r"[`*_~]", "", line)
        if len(line) >= 8:
            return line[:limit].rstrip("，,。；; ") + ("…" if len(line) > limit else "")
    return "最近更新的技术笔记。"


def _title(text: str, path: Path) -> str:
    value = _front_matter_value(text, "title")
    if value:
        return value
    m = _H1.search(_FRONT_MATTER.sub("", text, count=1))
    if m:
        return re.sub(r"[`*_~]", "", m.group(1)).strip()
    return path.stem


def _git_dates(repo_root: Path) -> Dict[str, Tuple[int, str]]:
    """Return newest commit timestamp/date for each docs file from local Git history."""
    try:
        p = subprocess.run(
            [
                "git", "-c", "core.quotepath=false", "log",
                "--format=@@COMMIT@@%ct|%cs", "--name-only", "--", "docs"
            ],
            cwd=repo_root,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=8,
            check=False,
        )
    except Exception:
        return {}
    if p.returncode != 0 or not p.stdout:
        return {}

    result: Dict[str, Tuple[int, str]] = {}
    current: Tuple[int, str] | None = None
    for raw in p.stdout.splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith("@@COMMIT@@"):
            try:
                stamp, date = line[len("@@COMMIT@@"):].split("|", 1)
                current = (int(stamp), date)
            except Exception:
                current = None
            continue
        if current and line.startswith("docs/") and line.endswith(".md") and line not in result:
            result[line] = current
    return result


def _github_repo_name(config) -> str:
    env_repo = os.getenv("GITHUB_REPOSITORY", "").strip()
    if "/" in env_repo:
        return env_repo
    repo_url = str(config.get("repo_url") or "")
    m = re.search(r"github\.com/([^/]+/[^/#]+)", repo_url)
    if m:
        return m.group(1).removesuffix(".git")
    return "qingyaozhuozhang/qingyaozhuozhang.github.io"


def _github_dates(config, wanted: int = 8) -> Dict[str, Tuple[int, str]]:
    """Fallback for shallow CI clones: read recent changed files from GitHub API."""
    repo = _github_repo_name(config)
    token = os.getenv("GITHUB_TOKEN", "").strip()
    headers = {
        "Accept": "application/vnd.github+json",
        "User-Agent": "mkdocs-recent-updates-hook",
    }
    if token:
        headers["Authorization"] = f"Bearer {token}"

    def get_json(url: str):
        req = urllib.request.Request(url, headers=headers)
        with urllib.request.urlopen(req, timeout=4) as r:
            return json.loads(r.read().decode("utf-8"))

    result: Dict[str, Tuple[int, str]] = {}
    try:
        commits = get_json(f"https://api.github.com/repos/{repo}/commits?per_page=15")
        for commit in commits:
            sha = commit.get("sha")
            if not sha:
                continue
            detail = get_json(f"https://api.github.com/repos/{repo}/commits/{sha}")
            iso = (((detail.get("commit") or {}).get("committer") or {}).get("date") or
                   ((detail.get("commit") or {}).get("author") or {}).get("date"))
            if not iso:
                continue
            dt = datetime.fromisoformat(iso.replace("Z", "+00:00"))
            stamp = int(dt.timestamp())
            date = dt.date().isoformat()
            for f in detail.get("files") or []:
                name = str(f.get("filename") or "")
                if name.startswith("docs/") and name.endswith(".md") and name not in result:
                    result[name] = (stamp, date)
            if len(result) >= wanted:
                break
    except Exception:
        return {}
    return result


def _collect_recent(config, limit: int = 5) -> Iterable[Tuple[int, str, str, str, str]]:
    docs_dir = Path(config["docs_dir"]).resolve()
    repo_root = docs_dir.parent
    date_map = _git_dates(repo_root)

    # A shallow GitHub Actions checkout may know only one commit. If there are too
    # few dated pages, supplement the data from GitHub's public API.
    if len(date_map) < limit:
        for k, v in _github_dates(config, wanted=limit + 3).items():
            date_map.setdefault(k, v)

    rows = []
    for path in docs_dir.rglob("*.md"):
        rel = path.relative_to(docs_dir).as_posix()
        if rel in {"index.md", "tags.md"} or path.name.startswith("."):
            continue
        try:
            text = path.read_text(encoding="utf-8-sig")
        except Exception:
            continue

        git_key = f"docs/{rel}"
        if git_key in date_map:
            stamp, date = date_map[git_key]
        else:
            stamp = int(path.stat().st_mtime)
            date = datetime.fromtimestamp(stamp).date().isoformat()

        rows.append((stamp, date, _title(text, path), rel, _plain_summary(text)))

    rows.sort(key=lambda x: (x[0], x[2]), reverse=True)
    return rows[:limit]


def _recent_markdown(config) -> str:
    rows = list(_collect_recent(config))
    if not rows:
        body = "> 暂时没有可显示的最近更新。"
    else:
        items = []
        for _, date, title, rel, summary in rows:
            items.append(f"- **{date} · [{title}]({rel})**  \n  {summary}")
        body = "\n".join(items)

    return (
        "<!-- AUTO_RECENT_UPDATES_START -->\n"
        f"{body}\n\n"
        "<small>由 Git 提交记录自动生成；新增或修改文章后，下次构建会自动更新。</small>\n"
        "<!-- AUTO_RECENT_UPDATES_END -->"
    )


def on_page_markdown(markdown, page, config, files):
    global _CACHE

    # Hide maintenance metadata on every rendered page, including future notes.
    markdown = MAINTENANCE_LINE.sub("", markdown)

    if getattr(page.file, "src_uri", "") == "index.md" and AUTO_BLOCK.search(markdown):
        if _CACHE is None:
            _CACHE = _recent_markdown(config)
        markdown = AUTO_BLOCK.sub(_CACHE, markdown)
    return markdown
