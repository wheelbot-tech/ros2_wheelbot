#!/bin/bash

# Adaugă toate modificările
git add .

# Creează un commit cu mesajul transmis ca argument
if [ -z "$1" ]; then
    echo "Introdu un mesaj pentru commit:"
    read commit_message
else
    commit_message=$1
fi

git commit -m "$commit_message"

# Push către ramura main
git push origin main
