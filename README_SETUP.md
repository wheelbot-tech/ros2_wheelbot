# WheelBot – Setup pentru proiect personal GitHub (fără PAT)

Acest pachet configurează workflow-ul GitHub Actions pentru a adăuga automat Issues & PR-uri
în proiectul personal de tip Projects v2:
🔗 https://github.com/users/wheelbot-tech/projects/2

## Pași de instalare
1️⃣ Copiază folderul `.github/workflows/` în rădăcina repo-ului `ros2_wheelbot`.
2️⃣ Fă commit & push:
    git add .github/workflows
    git commit -m "Add workflow for personal GitHub project (no PAT needed)"
    git push origin main

3️⃣ Creează un issue (ex. din template-ul Feature Request) și etichetează-l cu:
    workstream/software-ros2

4️⃣ Verifică după câteva secunde că apare în proiectul:
    https://github.com/users/wheelbot-tech/projects/2

Workflow-ul folosește `github.token` implicit, deci **nu necesită PAT** sau `PROJECT_TOKEN`.
Poți modifica lista de etichete urmărite în secțiunea `labeled:` din fișierul workflow.
