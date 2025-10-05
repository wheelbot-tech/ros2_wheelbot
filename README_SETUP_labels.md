# WheelBot – Auto Label Sync Pack

Acest pachet creează și sincronizează automat toate etichetele standard (workstream, type, variant, trl, priority) în repo-ul `ros2_wheelbot`.

## Instalare rapidă

1️⃣ Copiază conținutul acestui zip în rădăcina repo-ului `ros2_wheelbot`:
```
.github/labels.yml
.github/workflows/labels_sync.yml
```

2️⃣ Fă commit & push:
```bash
git add .github
git commit -m "Add GitHub label sync workflow"
git push origin main
```

3️⃣ Workflow-ul `Sync labels` va rula automat la primul push și va crea toate etichetele.

📍 Poți rula manual din tab-ul **Actions → Sync labels → Run workflow**.
