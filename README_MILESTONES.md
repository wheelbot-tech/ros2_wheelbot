# WheelBot – Milestones Pack

Acest pachet îți oferă **două moduri** de a crea/menține milestone-urile în repo:

1) **Workflow automat (recomandat):**
   - Fișier: `.github/workflows/milestones_sync.yml`
   - Config: `.github/milestones.yml`
   - La fiecare push pe `main` care modifică `.github/milestones.yml`, workflow-ul sincronizează milestones (creează/actualizează).

2) **Script local cu GitHub CLI (one-shot sau repetabil):**
   - Fișier: `scripts/create_milestones.sh`
   - Rulează: `bash scripts/create_milestones.sh wheelbot-tech/ros2_wheelbot`

## Instalare
1. Copiază în repo:
```
.github/milestones.yml
.github/workflows/milestones_sync.yml
scripts/create_milestones.sh
```
2. Commit & push:
```
git add .github scripts
git commit -m "Add milestones config + sync workflow"
git push origin main
```
3. (Opțional) Creează milestone-urile imediat via CLI:
```
bash scripts/create_milestones.sh wheelbot-tech/ros2_wheelbot
```

## Editare milestones
- Editează `.github/milestones.yml` și schimbă `title`, `due_on` (YYYY-MM-DD) sau `description`.
- Fă push pe `main` — workflow-ul le va sincroniza.

Succes! 🚀
