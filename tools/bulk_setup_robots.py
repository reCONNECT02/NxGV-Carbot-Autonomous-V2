#!/usr/bin/env python3
"""
RISA-bot Bulk Setup & Configuration Utility
=============================================================================
Automates running setup_mdns.sh, setup_wifi.sh, and install_bashalias.sh
across multiple robots simultaneously via SSH using paramiko (no password prompts).

Requirements:
  pip install paramiko

Usage:
  python tools/bulk_setup_robots.py
"""

import sys
import os
import time

# ANSI color codes
GREEN  = "\033[92m"
YELLOW = "\033[93m"
RED    = "\033[91m"
BLUE   = "\033[94m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

# ─────────────────────────────────────────────────
# Ensure paramiko is available
# ─────────────────────────────────────────────────
try:
    import paramiko
except ImportError:
    print(f"{YELLOW}paramiko not found. Installing...{RESET}")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "paramiko"])
    import paramiko

def print_banner():
    print(f"{CYAN}{BOLD}")
    print("======================================================")
    print("      🤖 RISA-Bot Multi-Robot Setup Utility 🤖")
    print("======================================================")
    print(f"{RESET}")

def ping_check(host):
    """Pings host to check if it is online."""
    import subprocess as sp
    param        = '-n' if sys.platform.lower().startswith('win') else '-c'
    timeout_param = '-w' if sys.platform.lower().startswith('win') else '-W'
    timeout_val   = '1000' if sys.platform.lower().startswith('win') else '1'
    try:
        res = sp.run(
            ['ping', param, '1', timeout_param, timeout_val, host],
            stdout=sp.DEVNULL, stderr=sp.DEVNULL
        )
        return res.returncode == 0
    except Exception:
        return False

def ssh_run(client, command, timeout=120):
    """
    Executes a command on an already-connected paramiko SSHClient.
    Returns (exit_code, stdout_str, stderr_str).
    """
    stdin, stdout, stderr = client.exec_command(command, timeout=timeout, get_pty=True)
    stdin.close()
    out = stdout.read().decode('utf-8', errors='replace')
    err = stderr.read().decode('utf-8', errors='replace')
    code = stdout.channel.recv_exit_status()
    return code, out, err

def configure_robot(host_ip, index, ssid, wifi_password, ssh_password):
    hostname = f"risabot{index}"
    print(f"\n{BOLD}{BLUE}[*] Configuring robot {index} ({host_ip}) -> Hostname: {hostname}...{RESET}")

    # ── 1. Connectivity check ──────────────────────────────────────────────
    print(f"  ⏳ Testing ping connection...")
    if not ping_check(host_ip):
        print(f"  {RED}❌ Link Down: {host_ip} is unreachable. Skipping.{RESET}")
        return False
    print(f"  {GREEN}✅ Link Up!{RESET}")

    # ── 2. Open SSH connection ─────────────────────────────────────────────
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    try:
        print(f"  ⏳ Connecting via SSH...")
        client.connect(
            hostname=host_ip,
            username='sunrise',
            password=ssh_password,
            timeout=15,
            banner_timeout=30,
            auth_timeout=20,
            look_for_keys=False,
            allow_agent=False
        )
        print(f"  {GREEN}✅ SSH Connected!{RESET}")
    except Exception as e:
        print(f"  {RED}❌ SSH Connection Failed: {e}{RESET}")
        return False

    success = True

    try:
        # ── Step 1: Git pull ───────────────────────────────────────────────
        print(f"  ⏳ [1/4] Pulling latest repository updates...")
        cmd = (
            "cd ~/risabotcar_ws && "
            "git fetch origin && "
            "git checkout refactor-test && "
            "git reset --hard origin/refactor-test"
        )
        code, out, err = ssh_run(client, cmd, timeout=120)
        if code != 0:
            print(f"  {YELLOW}⚠️  Git pull warning (code {code}), continuing...{RESET}")
        else:
            print(f"  {GREEN}✅ Repository updated!{RESET}")

        # ── Step 2: Bash aliases ───────────────────────────────────────────
        print(f"  ⏳ [2/4] Installing bash aliases...")
        code, out, err = ssh_run(client, "cd ~/risabotcar_ws && bash tools/install_bashalias.sh", timeout=60)
        if code != 0:
            print(f"  {YELLOW}⚠️  Alias install warning, continuing...{RESET}")
        else:
            print(f"  {GREEN}✅ Bash aliases configured!{RESET}")

        # ── Step 3: mDNS hostname ──────────────────────────────────────────
        print(f"  ⏳ [3/4] Setting hostname to '{hostname}'...")
        # Use -S so sudo reads password from stdin (piped); get_pty=True makes it non-interactive
        mdns_cmd = (
            f"echo '{ssh_password}' | sudo -S bash ~/risabotcar_ws/tools/setup_mdns.sh {hostname}"
        )
        code, out, err = ssh_run(client, mdns_cmd, timeout=60)
        if code != 0:
            print(f"  {RED}❌ mDNS setup failed (code {code}):{RESET}")
            print(f"     {err.strip()}")
            success = False
        else:
            print(f"  {GREEN}✅ Hostname set to '{hostname}', mDNS enabled!{RESET}")
            print(f"     -> Accessible at http://{hostname}.local:8080")

        # ── Step 4: WiFi ───────────────────────────────────────────────────
        print(f"  ⏳ [4/4] Configuring WiFi SSID '{ssid}'...")
        wifi_cmd = (
            f"echo '{ssh_password}' | sudo -S bash ~/risabotcar_ws/tools/setup_wifi.sh '{ssid}' '{wifi_password}' '{hostname}'"
        )
        code, out, err = ssh_run(client, wifi_cmd, timeout=60)
        if code != 0:
            print(f"  {RED}❌ WiFi setup failed (code {code}):{RESET}")
            print(f"     {err.strip()}")
            success = False
        else:
            print(f"  {GREEN}✅ WiFi profiles configured for SSID '{ssid}'!{RESET}")

        # ── Step 5: Reboot ─────────────────────────────────────────────────
        print(f"  ⏳ [5/5] Rebooting robot so hostname takes effect...")
        try:
            reboot_cmd = f"echo '{ssh_password}' | sudo -S reboot"
            client.exec_command(reboot_cmd)  # fire-and-forget, no wait
            print(f"  {GREEN}✅ Reboot command sent! Robot will come back as {hostname}.local{RESET}")
        except Exception:
            pass  # connection drops on reboot — this is expected

    except Exception as e:
        print(f"  {RED}❌ Error during configuration: {e}{RESET}")
        success = False
    finally:
        client.close()

    if success:
        print(f"{GREEN}{BOLD}🎉 Success: {host_ip} configured as {hostname}!{RESET}")
    else:
        print(f"{YELLOW}⚠️  {host_ip} partially configured as {hostname} (check errors above).{RESET}")

    return success


def main():
    print_banner()

    # ── Gather inputs ──────────────────────────────────────────────────────
    ssid = input(f"{BOLD}Enter WiFi SSID (Hotspot Name): {RESET}").strip()
    if not ssid:
        print("SSID cannot be empty. Exit.")
        return

    wifi_password = input(f"{BOLD}Enter WiFi Password: {RESET}").strip()
    if len(wifi_password) < 8:
        print("Password must be at least 8 characters. Exit.")
        return

    # SSH password is hardcoded to 'sunrise' (default for all robots)
    ssh_password = "sunrise"
    print(f"{CYAN}SSH password is set to 'sunrise' for all robots.{RESET}")

    print(f"\n{BOLD}Enter robot IP addresses (separated by commas):{RESET}")
    print("Example: 192.168.68.57, 192.168.68.58, 192.168.68.61")
    ip_input = input("> ").strip()
    if not ip_input:
        print("No IPs entered. Exit.")
        return

    robot_ips = [ip.strip() for ip in ip_input.split(',') if ip.strip()]

    print(f"\n{BOLD}Enter starting index for hostnames:{RESET}")
    print("(e.g. '1' -> risabot1, risabot2 ... '5' -> risabot5, risabot6 ...)")
    start_idx_str = input("Starting Index [default 1]: ").strip() or "1"
    try:
        start_idx = int(start_idx_str)
    except ValueError:
        print("Invalid index. Exit.")
        return

    # ── Summary ────────────────────────────────────────────────────────────
    print(f"\n{BOLD}{CYAN}Summary of Planned Operations:{RESET}")
    print(f"  WiFi SSID:     {ssid}")
    print(f"  WiFi Password: {'*' * len(wifi_password)}")
    print(f"  SSH Password:  sunrise (hardcoded)")
    print(f"  Target Robots:")
    for idx, ip in enumerate(robot_ips):
        print(f"    - {ip} -> risabot{start_idx + idx}  (http://risabot{start_idx + idx}.local:8080)")

    confirm = input(f"\n{BOLD}Proceed with configuring {len(robot_ips)} robots? (y/n): {RESET}").strip().lower()
    if confirm != 'y':
        print("Cancelled.")
        return

    # ── Execute ────────────────────────────────────────────────────────────
    success_count = 0
    start_time = time.time()

    for idx, ip in enumerate(robot_ips):
        target_idx = start_idx + idx
        if configure_robot(ip, target_idx, ssid, wifi_password, ssh_password):
            success_count += 1

    elapsed = time.time() - start_time
    print(f"\n{BOLD}======================================================{RESET}")
    print(f"Bulk Configuration Completed in {elapsed:.1f} seconds.")
    color = GREEN if success_count == len(robot_ips) else YELLOW
    print(f"Status: {color}{success_count}/{len(robot_ips)} Successful{RESET}")
    print(f"{BOLD}======================================================{RESET}")
    print()
    print(f"{YELLOW}⚠️  All robots are rebooting. Wait ~30 seconds, then SSH using:{RESET}")
    for idx, ip in enumerate(robot_ips):
        n = start_idx + idx
        print(f"  ssh sunrise@risabot{n}.local   (or via IP: ssh sunrise@{ip})")
    print()
    print(f"{CYAN}Dashboard accessible at:{RESET}")
    for idx, ip in enumerate(robot_ips):
        print(f"  http://risabot{start_idx + idx}.local:8080")


if __name__ == '__main__':
    main()
