# AWS RMF skeleton deployment

This directory is the EC2 deployment surface for the WheelBot Open-RMF stack.

Stage 2 is intentionally small. It validates:

- Docker Compose on the EC2 instance;
- DNS for `fleet.wheelbot.tech` and `api.fleet.wheelbot.tech`;
- Caddy automatic HTTPS;
- reverse proxy routing;
- a persistent PostgreSQL volume.

It does not yet run Zenoh, ROS 2, Open-RMF, RMF Web, or the WheelBot fleet adapter.

## Services

```text
caddy
  publishes :80 and :443
  terminates HTTPS
  routes fleet.wheelbot.tech -> dashboard-smoke
  routes api.fleet.wheelbot.tech -> api-smoke

dashboard-smoke
  temporary test web service

api-smoke
  temporary test API service

postgres
  persistent PostgreSQL database for later RMF Web API use
```

Only Caddy publishes host ports. PostgreSQL is not exposed publicly.

## First deploy on EC2

From the EC2 SSM session:

```bash
cd /home/ubuntu/ros2_wheelbot/deploy/aws-rmf
cp .env.example .env
```

Edit `.env` on EC2 and change at least:

```text
POSTGRES_PASSWORD=...
ACME_EMAIL=...
```

Start the skeleton:

```bash
docker compose pull
docker compose up -d
```

Check containers:

```bash
docker compose ps
docker compose logs caddy
```

## Validation from your laptop

```bash
curl -I https://fleet.wheelbot.tech
curl -I https://api.fleet.wheelbot.tech/health
curl https://api.fleet.wheelbot.tech/health
```

Expected:

- HTTPS certificates are issued by Caddy/Let's Encrypt.
- `https://fleet.wheelbot.tech` returns the dashboard smoke service.
- `https://api.fleet.wheelbot.tech/health` returns `ok`.

## Update flow

Local laptop:

```bash
git add deploy/aws-rmf
git commit -m "Add AWS RMF skeleton deployment"
git push
```

EC2:

```bash
cd /home/ubuntu/ros2_wheelbot
git pull
cd deploy/aws-rmf
docker compose pull
docker compose up -d
```

## Stop/start

Stop only the skeleton containers:

```bash
cd /home/ubuntu/ros2_wheelbot/deploy/aws-rmf
docker compose down
```

Start them again:

```bash
docker compose up -d
```

To stop EC2 compute billing, stop the EC2 instance from the AWS Console. EBS and public IPv4/Elastic IP related charges can still apply.

## Stage 3 replacement points

Later, Stage 3 will replace the smoke services with:

- local Zenoh router connected outbound to ZettaScale Cloud;
- ROS 2 Kilted containers using `rmw_zenoh_cpp`;
- Open-RMF core;
- RMF Web API image compatible with the selected ROS/RMW stack.

