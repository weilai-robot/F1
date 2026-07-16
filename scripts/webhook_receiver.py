#!/usr/bin/env python3
"""
webhook_receiver.py — GitHub Webhook Receiver (workflow_run 事件)

部署在 Agent 机器上，监听 GitHub 推送的 workflow_run completed 事件。
CI 跑完后自动触发结果下载 + 分析。

架构:
  GitHub Actions 跑完 → GitHub 发 Webhook → 本脚本收到 → 下载 artifact → 分析

启动:
  python3 scripts/webhook_receiver.py --port 9090 --secret <webhook_secret>

  # 或用环境变量
  export WEBHOOK_SECRET=mysecret
  python3 scripts/webhook_receiver.py

GitHub 配置 (Settings → Webhooks → Add webhook):
  URL:           http://<agent-ip>:9090/webhook
  Content type:  application/json
  Secret:        <与启动参数一致>
  Events:        ✓ Workflow runs

回调:
  收到结果后自动执行 scripts/fetch_ci_results.py
  可通过 --callback 指定自定义回调命令
"""
import argparse
import hashlib
import hmac
import json
import os
import subprocess
import sys
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path

# ── 颜色 ──────────────────────────────────────────────────
G = "\033[0;32m"
Y = "\033[1;33m"
R = "\033[0;31m"
B = "\033[0;34m"
N = "\033[0m"


class WebhookHandler(BaseHTTPRequestHandler):
    """处理 GitHub Webhook POST"""

    # 类级配置 (由 main 注入)
    SECRET = ""
    CALLBACK_CMD = ""
    REPO_FILTER = ""       # 只处理此 repo (如 "weilai-robot/F1")
    WORKFLOW_FILTER = ""   # 只处理此 workflow (如 "Nav Eval")
    SCRIPTS_DIR = ""

    def log_message(self, format, *args):
        """覆盖默认日志，加时间戳"""
        ts = time.strftime("%H:%M:%S")
        print(f"  [{ts}] {args[0] % args[1:]}")

    def do_POST(self):
        content_length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_length)

        # ── 1. 验证签名 ──
        signature = self.headers.get("X-Hub-Signature-256", "")
        if not self._verify_signature(body, signature):
            self.send_response(403)
            self.end_headers()
            print(f"{R}[webhook] ✗ 签名验证失败{N}")
            return

        # ── 2. 解析事件类型 ──
        event_type = self.headers.get("X-GitHub-Event", "")
        payload = json.loads(body)

        # ── 3. ping 事件 (webhook 首次配置时) ──
        if event_type == "ping":
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'{"status":"ok","msg":"pong"}')
            print(f"{G}[webhook] ✓ Ping received — webhook 配置成功{N}")
            return

        # ── 4. workflow_run 事件 ──
        if event_type != "workflow_run":
            self.send_response(200)
            self.end_headers()
            print(f"  [webhook] 忽略事件: {event_type}")
            return

        action = payload.get("action", "")
        if action != "completed":
            self.send_response(200)
            self.end_headers()
            print(f"  [webhook] workflow_run action={action}, 忽略 (只处理 completed)")
            return

        # ── 5. 过滤仓库 + workflow ──
        repo = payload.get("repository", {}).get("full_name", "")
        if self.REPO_FILTER and repo != self.REPO_FILTER:
            self.send_response(200)
            self.end_headers()
            print(f"  [webhook] 忽略仓库: {repo} (filter: {self.REPO_FILTER})")
            return

        workflow_name = payload.get("workflow", {}).get("name", "")
        if self.WORKFLOW_FILTER and workflow_name != self.WORKFLOW_FILTER:
            self.send_response(200)
            self.end_headers()
            print(f"  [webhook] 忽略 workflow: {workflow_name}")
            return

        # ── 6. 提取 run 信息 ──
        run = payload.get("workflow_run", {})
        run_id = run.get("id", 0)
        conclusion = run.get("conclusion", "unknown")
        branch = run.get("head_branch", "?")
        run_number = run.get("run_number", 0)
        html_url = run.get("html_url", "")

        print(f"\n{G}{'='*60}{N}")
        print(f"{G}[webhook] ✓ Nav Eval CI 完成!{N}")
        print(f"  run_id:     {run_id}")
        print(f"  run_number: #{run_number}")
        print(f"  branch:     {branch}")
        print(f"  conclusion: {conclusion}")
        print(f"  url:        {html_url}")
        print(f"{G}{'='*60}{N}")

        # 立即返回 200 (GitHub 要求快速响应, 否则超时重试)
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps({
            "status": "accepted",
            "run_id": run_id,
            "triggering_download": True,
        }).encode())

        # ── 7. 异步下载 + 分析 ──
        thread = threading.Thread(
            target=self._download_and_analyze,
            args=(run_id, run_number, branch, conclusion),
            daemon=True,
        )
        thread.start()

    def _verify_signature(self, body: bytes, signature: str) -> bool:
        """验证 GitHub Webhook 签名 (HMAC-SHA256)"""
        if not self.SECRET:
            # 未配置 secret, 跳过验证 (仅开发模式)
            return True

        if not signature.startswith("sha256="):
            return False

        expected = hmac.new(
            self.SECRET.encode(),
            body,
            hashlib.sha256,
        ).hexdigest()

        received = signature[7:]  # 去掉 "sha256=" 前缀
        return hmac.compare_digest(expected, received)

    def _download_and_analyze(self, run_id, run_number, branch, conclusion):
        """异步: 下载 artifact + 分析 + 触发回调"""
        print(f"\n{B}[fetch] 下载 run {run_id} 的结果...{N}")

        dest = os.path.abspath("ci_fetched")
        os.makedirs(dest, exist_ok=True)

        # 下载 artifact
        result = subprocess.run(
            ["gh", "run", "download", str(run_id),
             f"--name=nav-results-{run_id}",
             f"--dir={dest}"],
            capture_output=True, text=True,
        )

        if result.returncode != 0:
            print(f"{R}[fetch] ✗ 下载失败: {result.stderr[:300]}{N}")
            # 尝试不带 name 下载 (可能 artifact 名不同)
            result = subprocess.run(
                ["gh", "run", "download", str(run_id), f"--dir={dest}"],
                capture_output=True, text=True,
            )
            if result.returncode != 0:
                print(f"{R}[fetch] ✗ 再次下载失败: {result.stderr[:300]}{N}")
                return

        print(f"{G}[fetch] ✓ artifact 下载到 {dest}/{N}")

        # 分析结果
        fetch_script = os.path.join(self.SCRIPTS_DIR, "fetch_ci_results.py")
        if os.path.exists(fetch_script):
            print(f"{B}[fetch] 分析结果...{N}")
            analyze = subprocess.run(
                ["python3", fetch_script,
                 "--run-id", str(run_id),
                 "--no-wait",
                 "--dest", dest],
                capture_output=True, text=True,
            )
            report_path = os.path.join(dest, "report.md")
            if os.path.exists(report_path):
                print(f"{G}[fetch] ✓ 诊断报告: {report_path}{N}")
                # 打印报告前 30 行
                with open(report_path) as f:
                    lines = f.readlines()[:30]
                    print("".join(lines))
            else:
                print(f"{Y}[fetch] 分析脚本未生成报告{N}")
                if analyze.stderr:
                    print(f"  stderr: {analyze.stderr[:300]}")

        # 触发自定义回调
        if self.CALLBACK_CMD:
            print(f"{B}[callback] 触发回调: {self.CALLBACK_CMD}{N}")
            env = os.environ.copy()
            env.update({
                "CI_RUN_ID": str(run_id),
                "CI_RUN_NUMBER": str(run_number),
                "CI_BRANCH": branch,
                "CI_CONCLUSION": conclusion,
                "CI_REPORT_DIR": dest,
                "CI_REPORT_FILE": os.path.join(dest, "report.md"),
            })
            subprocess.run(self.CALLBACK_CMD, shell=True, env=env)


def main():
    parser = argparse.ArgumentParser(
        description="GitHub Webhook Receiver for Nav Eval CI"
    )
    parser.add_argument("--port", type=int, default=9090, help="监听端口")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="监听地址")
    parser.add_argument("--secret", type=str, default=os.environ.get("WEBHOOK_SECRET", ""),
                        help="Webhook 密钥 (用于验证 GitHub 签名)")
    parser.add_argument("--repo", type=str, default="weilai-robot/F1",
                        help="只处理此仓库的 webhook")
    parser.add_argument("--workflow", type=str, default="Nav Eval",
                        help="只处理此 workflow")
    parser.add_argument("--callback", type=str, default="",
                        help="收到结果后的自定义回调命令 "
                             "(环境变量: CI_RUN_ID, CI_BRANCH, CI_CONCLUSION, CI_REPORT_FILE)")
    parser.add_argument("--scripts-dir", type=str, default="scripts",
                        help="脚本目录 (fetch_ci_results.py 所在位置)")
    args = parser.parse_args()

    # 注入配置到 Handler 类
    WebhookHandler.SECRET = args.secret
    WebhookHandler.CALLBACK_CMD = args.callback
    WebhookHandler.REPO_FILTER = args.repo
    WebhookHandler.WORKFLOW_FILTER = args.workflow
    WebhookHandler.SCRIPTS_DIR = os.path.abspath(args.scripts_dir)

    # 检查 gh CLI
    gh_path = subprocess.run(["which", "gh"], capture_output=True, text=True).stdout.strip()
    if not gh_path:
        print(f"{R}[ERROR] gh CLI 未安装{N}")
        print(f"  安装: https://cli.github.com/")
        sys.exit(1)

    # 启动
    print(f"{G}{'='*60}{N}")
    print(f"{G} GitHub Webhook Receiver — Nav Eval CI{N}")
    print(f"{G}{'='*60}{N}")
    print(f"  监听:       {args.host}:{args.port}")
    print(f"  仓库过滤:   {args.repo}")
    print(f"  Workflow:   {args.workflow}")
    print(f"  签名验证:   {'✓ 已启用' if args.secret else '✗ 未配置 (仅开发模式)'}")
    print(f"  回调命令:   {args.callback or '(无)'}")
    print(f"  gh CLI:     {gh_path}")
    print(f"  GitHub 配置:")
    print(f"    URL: http://<agent-ip>:{args.port}/webhook")
    print(f"    (Settings → Webhooks → Add webhook)")
    print(f"{G}{'='*60}{N}")
    print(f"\n等待 webhook...\n")

    server = HTTPServer((args.host, args.port), WebhookHandler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print(f"\n{Y}[webhook] 已停止{N}")
        server.server_close()


if __name__ == "__main__":
    main()
