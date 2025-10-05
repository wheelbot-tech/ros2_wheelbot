#!/bin/bash
# ...existing code...

# Adaugă toate modificările
git add .

# Creează un commit cu mesajul transmis ca argument
if [ -z "$1" ]; then
    echo "Introdu un mesaj pentru commit:"
    read commit_message
else
    commit_message=$1
fi

# Commit doar dacă sunt schimbări pregătite
if git diff --cached --quiet --exit-code; then
    echo "Nu sunt schimbări de comis."
else
    git commit -m "$commit_message" || { echo "Commit eșuat."; exit 1; }
fi

# ...existing code...
# Push către branch-ul curent, după rebase cu remote
current_branch=$(git rev-parse --abbrev-ref HEAD)

# Fetch și rebase; oprește la conflicte pentru a le rezolva manual
git fetch origin "$current_branch" || { echo "Eroare la fetch."; exit 1; }
git rebase origin/"$current_branch" || { echo "Rebase eșuat. Rezolvă conflictele, apoi rulează 'git rebase --continue' sau 'git rebase --abort'."; exit 1; }

git push -u origin "$current_branch" || { echo "Push eșuat. Dacă știi ce faci, poți încerca 'git push --force-with-lease'."; exit 1; }