# AWS + ZettaScale Cloud + Open-RMF deployment plan

Scop: WheelBot ruleaza controlul local, Nav2 si siguranta pe robot/Balena, ZettaScale Cloud este backbone-ul Zenoh managed, iar AWS EC2 gazduieste Open-RMF, RMF Web API, dashboard-ul si baza de date.

Arhitectura tinta:

```text
WheelBot / Balena / Jetson
  base_control + Nav2 local + rmf_agent / robot handler
  local Zenoh router
        |
        | outbound mTLS / QUIC / TLS
        v
ZettaScale Cloud
  managed Zenoh router / backbone
        |
        | outbound mTLS / QUIC / TLS
        v
AWS EC2
  local Zenoh router / rmw_zenoh support
  Open-RMF core
  WheelBot fleet adapter / robot integration
  RMF Web API
  dashboard
  PostgreSQL
  Caddy / HTTPS reverse proxy
```

Principii:

- ZettaScale Cloud transporta doar traficul ROS/RMF necesar pentru coordonare, nu stream-uri grele de senzori.
- EC2 nu expune Zenoh public pe Internet.
- Browserul comunica prin HTTPS cu RMF Web API/dashboard, nu direct cu ROS/Zenoh.
- Robotul pastreaza local Nav2, obstacle avoidance, timeout-uri, ESTOP si comportamentul sigur la pierderea conexiunii.
- Toti robotii care trebuie vazuti de acelasi Open-RMF/fleet adapter folosesc acelasi `ROS_DOMAIN_ID`; separarea robotilor se face prin namespace ROS, de exemplu `/robot_1`, `/robot_2`.

Variabile ROS recomandate pentru componentele care trebuie sa fie in acelasi graf:

```bash
RMW_IMPLEMENTATION=rmw_zenoh_cpp
ROS_DOMAIN_ID=42
```

`ROS_DOMAIN_ID=42` este un exemplu. Poate fi si `0`, dar trebuie sa fie acelasi pe robot, EC2 si procesele RMF care trebuie sa comunice intre ele.

---

## Stage 1 — AWS foundation

Aceasta etapa se face manual din AWS Dashboard. Nu include inca deploy Docker, build imagini, Open-RMF sau conectarea Zenoh la ZettaScale.

Rezultatul dorit la finalul Stage 1:

- EC2 Ubuntu 24.04 este pornit si accesibil prin AWS Systems Manager Session Manager.
- Elastic IP este atasat instantei.
- DNS-ul pentru `fleet.wheelbot.tech` si `api.fleet.wheelbot.tech` pointeaza catre Elastic IP.
- Security Group-ul expune public doar serviciile web necesare.
- Instanta are IAM role pentru SSM si, optional, Route53 DNS challenge.
- Docker si Docker Compose plugin sunt instalate.
- Repo-ul este clonat pe instanta.

### 1.1 IAM role pentru EC2

In AWS Console:

- Mergi la IAM -> Roles -> Create role.
- Trusted entity: AWS service.
- Use case: EC2.
- Ataseaza policy:
  - `AmazonSSMManagedInstanceCore`

Optional, daca Caddy va folosi Route53 DNS challenge pentru certificate TLS wildcard sau certificate fara port 80:

```json
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": [
        "route53:ListHostedZones",
        "route53:GetChange",
        "route53:ChangeResourceRecordSets"
      ],
      "Resource": "*"
    }
  ]
}
```

Recomandare: pentru inceput poti folosi policy-ul Route53 de mai sus, apoi il restrangi la hosted zone-ul exact pentru `wheelbot.tech`.

Checklist:

- [ ] IAM role creat pentru EC2.
- [ ] `AmazonSSMManagedInstanceCore` atasat.
- [ ] Optional: Route53 DNS challenge policy atasata.

### 1.2 Security Group

Creeaza un Security Group dedicat pentru instanta Open-RMF.

Inbound recomandat:

| Port | Protocol | Source | Scop |
|------|----------|--------|------|
| 443 | TCP | `0.0.0.0/0`, `::/0` | HTTPS dashboard + API |
| 80 | TCP | `0.0.0.0/0`, `::/0` | optional, redirect HTTP -> HTTPS sau ACME HTTP challenge |

Nu expune public:

- `7447` Zenoh
- `8000` RMF Web API direct
- `3000` dashboard direct
- `5432` PostgreSQL
- SSH `22`, daca folosesti SSM Session Manager

Outbound:

- permite outbound HTTPS/TLS si conexiunea catre endpoint-ul ZettaScale Cloud.
- pentru inceput poate ramane default outbound allow-all; se poate restrange dupa ce endpoint-urile sunt stabilizate.

Checklist:

- [ ] Security Group creat.
- [ ] Inbound `443` public.
- [ ] Optional inbound `80` public.
- [ ] Fara inbound public pentru Zenoh/PostgreSQL/RMF API intern.
- [ ] SSH public dezactivat, daca SSM functioneaza.

### 1.3 EC2 instance

In AWS Console -> EC2 -> Launch instance:

- Name: `wheelbot-open-rmf`
- AMI: Ubuntu Server 24.04 LTS
- Instance type initial: `t3.xlarge` sau `t3.large`
  - `t3.xlarge` este mai confortabil pentru build-uri locale Docker/Open-RMF.
  - `t3.large` poate fi suficient dupa ce imaginile sunt deja construite.
- Storage: minim 30 GB gp3; recomandat 50-80 GB daca build-urile se fac pe instanta.
- IAM instance profile: rolul creat la pasul 1.1.
- Security Group: cel creat la pasul 1.2.
- Auto-assign public IP: poate fi activ temporar, dar Elastic IP va fi atasat dupa lansare.

Checklist:

- [ ] EC2 lansat.
- [ ] Status checks `2/2 passed`.
- [ ] IAM role atasat.
- [ ] Security Group corect atasat.

### 1.4 Elastic IP

In AWS Console:

- EC2 -> Elastic IPs -> Allocate Elastic IP.
- Associate Elastic IP cu instanta `wheelbot-open-rmf`.

Checklist:

- [ ] Elastic IP alocat.
- [ ] Elastic IP atasat instantei.
- [ ] IP-ul ramane stabil dupa restart.

### 1.5 Route53 DNS

In hosted zone-ul `wheelbot.tech`, creeaza:

| Record | Type | Value |
|--------|------|-------|
| `fleet.wheelbot.tech` | A | Elastic IP |
| `api.fleet.wheelbot.tech` | A | Elastic IP |

Optional, daca folosesti IPv6:

| Record | Type | Value |
|--------|------|-------|
| `fleet.wheelbot.tech` | AAAA | IPv6 instance address |
| `api.fleet.wheelbot.tech` | AAAA | IPv6 instance address |

Checklist:

- [ ] `fleet.wheelbot.tech` A record creat.
- [ ] `api.fleet.wheelbot.tech` A record creat.
- [ ] DNS rezolva catre Elastic IP.

Verificare de pe calculatorul local:

```bash
dig fleet.wheelbot.tech
dig api.fleet.wheelbot.tech
```

### 1.6 SSM Session Manager

In AWS Console:

- EC2 -> Instances -> selecteaza instanta.
- Apasa Connect.
- Alege Session Manager.

Daca nu apare disponibila conexiunea:

- verifica IAM role-ul `AmazonSSMManagedInstanceCore`;
- verifica daca instanta are outbound Internet/NAT;
- verifica daca SSM Agent ruleaza pe Ubuntu.

Checklist:

- [ ] Conexiune SSM functionala.
- [ ] Nu este necesar SSH public.

### 1.7 Bootstrap manual pe instanta

Conecteaza-te prin SSM Session Manager si instaleaza baza:

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg git unzip
```

Instaleaza Docker din repository-ul oficial Docker:

```bash
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME:-$VERSION_CODENAME}) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

Adauga utilizatorii locali relevanti in grupul `docker`:

```bash
id ubuntu && sudo usermod -aG docker ubuntu || true
id ssm-user && sudo usermod -aG docker ssm-user || true
```

Instaleaza AWS CLI v2:

```bash
curl "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o "/tmp/awscliv2.zip"
unzip -q /tmp/awscliv2.zip -d /tmp
sudo /tmp/aws/install
```

Reconecteaza sesiunea dupa `usermod`, apoi verifica:

```bash
docker --version
docker compose version
aws sts get-caller-identity
```

Cloneaza repo-ul:

```bash
cd /home/ubuntu
git clone <repo-url> ros2_wheelbot
cd ros2_wheelbot
```

Checklist:

- [ ] Docker instalat.
- [ ] Docker Compose plugin instalat.
- [ ] AWS CLI instalat.
- [ ] `aws sts get-caller-identity` functioneaza.
- [ ] Repo clonat in `/home/ubuntu/ros2_wheelbot`.

### 1.8 Stage 1 validation

De pe local:

```bash
dig fleet.wheelbot.tech
dig api.fleet.wheelbot.tech
```

Din SSM session pe EC2:

```bash
docker --version
docker compose version
aws route53 list-hosted-zones
```

Checklist final Stage 1:

- [ ] Instanta este `running`.
- [ ] Status checks sunt `2/2 passed`.
- [ ] SSM Session Manager functioneaza.
- [ ] Elastic IP este atasat.
- [ ] DNS rezolva corect.
- [ ] Security Group nu expune Zenoh/PostgreSQL/API intern.
- [ ] Docker + Compose sunt disponibile.
- [ ] Repo-ul este clonat.

---

## Stage 2 — Skeleton deployment pe EC2

Aceasta etapa introduce structura de deploy, fara integrare completa WheelBot/Open-RMF.

Rezultatul dorit:

- stack Docker Compose porneste pe EC2;
- Caddy obtine certificate HTTPS;
- dashboard si API sunt accesibile prin domeniile finale;
- PostgreSQL este persistent;
- secretele sunt in `.env`, nu in git.

Fisiere propuse in repo:

```text
deploy/aws-rmf/
  docker-compose.yml
  .env.example
  Caddyfile
  postgresql.conf
  README.md
```

Servicii initiale:

- `caddy`
- `postgres`
- `rmf-web-api`
- `rmf-dashboard`

Checklist:

- [ ] Creeaza `.env.example` fara secrete reale.
- [ ] Creeaza `docker-compose.yml`.
- [ ] Configureaza volume persistent pentru PostgreSQL.
- [ ] Configureaza Caddy pentru:
  - `https://fleet.wheelbot.tech`
  - `https://api.fleet.wheelbot.tech`
- [ ] Configureaza RMF Web API cu DB persistent.
- [ ] Configureaza health checks.

Validare:

```bash
docker compose ps
curl -I https://fleet.wheelbot.tech
curl -I https://api.fleet.wheelbot.tech/health
```

Note:

- Daca folosesti Cognito/OIDC din aceasta etapa, callback-urile trebuie sa includa domeniile finale.
- Daca RMF Web API nu are suport pentru `rmw_zenoh_cpp` in imaginea folosita, Stage 3 va introduce imagine custom.

---

## Stage 3 — Zenoh + ROS/Open-RMF runtime

Aceasta etapa conecteaza EC2 la ZettaScale Cloud si pregateste runtime-ul ROS/Open-RMF.

Rezultatul dorit:

- EC2 are local Zenoh router/daemon.
- EC2 se conecteaza outbound la ZettaScale Cloud.
- componentele ROS 2 de pe EC2 folosesc `rmw_zenoh_cpp`.
- un test ROS simplu confirma comunicatia prin ZettaScale Cloud.
- Open-RMF core porneste intr-o imagine compatibila cu ROS 2 Kilted.

Fisiere propuse:

```text
deploy/aws-rmf/
  zenoh/
    zenoh-router.json5
    certs/
      .gitkeep
  docker/
    Dockerfile.ros-kilted-rmw-zenoh
    Dockerfile.rmf-core
    Dockerfile.rmf-web-api
```

Reguli:

- Nu publica certificatul/token-ul ZettaScale in git.
- Nu expune Zenoh inbound public pe EC2.
- EC2 initiaza conexiunea outbound catre ZettaScale Cloud.
- Robotul/Balena initiaza separat conexiunea outbound catre ZettaScale Cloud.

Config ROS comuna:

```bash
RMW_IMPLEMENTATION=rmw_zenoh_cpp
ROS_DOMAIN_ID=42
```

Checklist:

- [ ] Creeaza proiect/router in ZettaScale Cloud.
- [ ] Genereaza credentiale/cert/token pentru EC2 peer.
- [ ] Genereaza credentiale/cert/token separat pentru robot/Balena.
- [ ] Configureaza Zenoh router pe EC2.
- [ ] Configureaza `rmw_zenoh_cpp` in containerele ROS/RMF.
- [ ] Testeaza topic ROS simplu intre robot/EC2 sau intre doua procese controlate.
- [ ] Porneste Open-RMF core.

Validare:

```bash
docker compose logs zenoh
docker compose logs rmf-core
ros2 topic list
ros2 node list
```

Note:

- Daca robotul este namespaced ca `/robot_1`, RMF/fleet adapter trebuie configurat explicit pentru acel namespace.
- Nu trimite prin ZettaScale Cloud topicuri grele precum camere, point clouds, costmaps sau `/scan`, decat daca exista un motiv clar.

---

## Stage 4 — WheelBot RMF integration

Aceasta etapa face integrarea reala WheelBot cu Open-RMF.

Rezultatul dorit:

- RMF vede robotul `robot_1`.
- Dashboard-ul poate trimite task-uri.
- Fleet adapter-ul converteste task-urile RMF in goal-uri Nav2 locale.
- Robotul raporteaza stare, pozitie, baterie si disponibilitate.
- Pierderea conexiunii este tratata sigur.

Componente:

```text
Open-RMF Core
  -> WheelBot fleet adapter
  -> robot command handle / robot handler
  -> /robot_1 Nav2 action server
  -> local WheelBot control
```

Checklist tehnic:

- [ ] Creeaza sau adapteaza fleet adapter pentru WheelBot.
- [ ] Configureaza `fleet_name`, `robot_name`, `namespace`.
- [ ] Creeaza nav graph RMF pentru harta WheelBot.
- [ ] Aliniaza coordonatele RMF map <-> Nav2 map.
- [ ] Configureaza robot profile/footprint.
- [ ] Configureaza task capabilities initiale:
  - go-to-place
  - patrol/demo
  - cancel/hold
- [ ] Publica robot state:
  - pose
  - mode
  - battery
  - current waypoint/destination
  - offline/online
- [ ] Integreaza RMF Web dashboard cu task creation/cancel.

Safety checklist:

- [ ] Robotul nu depinde de cloud pentru obstacle avoidance.
- [ ] Nav2 ruleaza local pe robot.
- [ ] La pierderea conexiunii, robotul opreste/hold local.
- [ ] La timeout de comanda RMF, goal-ul Nav2 este anulat sau pus in hold.
- [ ] ESTOP local ramane prioritar.
- [ ] Teleop/joy/twist_mux nu concureaza necontrolat cu RMF.

Validare:

- [ ] RMF vede `robot_1`.
- [ ] Dashboard arata robotul pe harta.
- [ ] Task simplu trimis din dashboard ajunge la Nav2.
- [ ] Cancel task functioneaza.
- [ ] Robot offline este reflectat corect in RMF.
- [ ] Reconectarea robotului nu creeaza duplicate de robot state.

---

## Decizii deschise

- Alegerea exacta a instance type dupa primele build-uri: `t3.large`, `t3.xlarge` sau alta familie.
- Daca imaginile Docker se construiesc pe EC2, local sau in CI.
- Daca PostgreSQL ramane container local pe EBS sau se muta ulterior in RDS.
- Daca autentificarea se face din start cu Cognito/OIDC sau dupa skeleton deployment.
- Daca integrarea initiala foloseste un adapter existent ca prototip sau se implementeaza direct un WheelBot-specific fleet adapter.

---

## Out of scope initial

- Kubernetes/EKS.
- Multi-AZ/high availability.
- RDS managed PostgreSQL.
- CI/CD complet.
- WAF/DDoS hardening.
- Monitoring complet Prometheus/Grafana/CloudWatch.
- Customizare ampla a UI-ului RMF Dashboard.
