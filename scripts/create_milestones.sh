#!/usr/bin/env bash
# Creează/actualizează milestones în repo folosind GitHub CLI.
# Usage: ./scripts/create_milestones.sh wheelbot-tech/ros2_wheelbot
set -euo pipefail

REPO="${1:-wheelbot-tech/ros2_wheelbot}"

# Necesită gh: https://cli.github.com/ ; Autentificare: gh auth login
if ! command -v gh >/dev/null 2>&1; then
  echo "Eroare: gh CLI nu este instalat. Instalează-l și rulează din nou."
  exit 1
fi

# Citește .github/milestones.yml din repo curent
if [ ! -f ".github/milestones.yml" ]; then
  echo "Nu am găsit .github/milestones.yml în directorul curent."
  exit 1
fi

python3 - <<'PY' "$REPO"
import sys, yaml, subprocess
repo = sys.argv[1]
with open(".github/milestones.yml", "r", encoding="utf-8") as f:
    data = yaml.safe_load(f)
for m in data.get("milestones", []):
    title = m["title"]
    due_on = m.get("due_on")
    desc = m.get("description", "")
    # Încearcă să creezi; dacă există, actualizează
    create = subprocess.run(["gh","api",
                             f"repos/{repo}/milestones",
                             "-f", f"title={title}",
                             *(["-f", f"due_on={due_on}T00:00:00Z"] if due_on else []),
                             "-f", f"description={desc}"],
                            capture_output=True, text=True)
    if create.returncode == 0:
        print(f"Created: {title}")
    else:
        # Obține ID-ul milestone-ului existent
        ls = subprocess.run(["gh","api", f"repos/{repo}/milestones?state=all"],
                            capture_output=True, text=True, check=True)
        import json
        arr = json.loads(ls.stdout)
        match = next((x for x in arr if x["title"] == title), None)
        if not match:
            print(f"Eșec creare: {title} -> {create.stderr.strip()}")
        else:
            number = str(match["number"])
            args = ["gh","api","--method","PATCH", f"repos/{repo}/milestones/{number}",
                    "-f", f"title={title}",
                    *(["-f", f"due_on={due_on}T00:00:00Z"] if due_on else []),
                    "-f", f"description={desc}"]
            subprocess.run(args, check=True)
            print(f"Updated: {title}")
PY
