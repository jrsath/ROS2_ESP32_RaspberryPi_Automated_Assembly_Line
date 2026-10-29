#!/usr/bin/env python3
# Forward one ESP32's UDP traffic between Windows Wi-Fi and the WSL agent.
# Run with Windows Python and PyYAML. Addresses come from ID.yaml.

import argparse
from pathlib import Path
import yaml
import ipaddress
import selectors
import socket
import subprocess


def main():
    identity = yaml.safe_load(
        (Path(__file__).resolve().parents[1] / "ID.yaml").read_text()
    )["network"]
    parser = argparse.ArgumentParser(description="ESP32 to WSL micro-ROS UDP relay")
    parser.add_argument(
        "--listen", default=identity["agent_ip"], help="Windows Wi-Fi IPv4 address"
    )
    parser.add_argument(
        "--esp-ip", default=identity["esp_ip"], help="ESP32 IPv4 from router DHCP list"
    )
    parser.add_argument("--port", type=int, default=identity["agent_port"])
    parser.add_argument(
        "--agent-host", help="WSL IPv4; detected automatically if omitted"
    )
    parser.add_argument("--agent-port", type=int, default=identity["agent_port"])
    args = parser.parse_args()
    # Accept literal IPv4 addresses so forwarding destinations are explicit.
    ipaddress.IPv4Address(args.listen)
    ipaddress.IPv4Address(args.esp_ip)
    if not args.agent_host:
        addresses = subprocess.check_output(
            ["wsl.exe", "-d", "Ubuntu-22.04", "hostname", "-I"], text=True
        ).split()
        args.agent_host = next(address for address in addresses if ":" not in address)
    ipaddress.IPv4Address(args.agent_host)
    if not (1 <= args.port <= 65535 and 1 <= args.agent_port <= 65535):
        parser.error("Ports must be between 1 and 65535")

    # Separate sockets keep the ESP32 and agent endpoints distinct.
    with (
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as board,
        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as agent,
        selectors.DefaultSelector() as selector,
    ):
        board.bind((args.listen, args.port))
        agent.connect((args.agent_host, args.agent_port))
        selector.register(board, selectors.EVENT_READ)
        selector.register(agent, selectors.EVENT_READ)
        peer = None
        print(
            f"ESP32 {args.esp_ip} -> {args.listen}:{args.port} -> "
            f"WSL {args.agent_host}:{args.agent_port}",
            flush=True,
        )
        print("Leave this terminal open. Ctrl+C stops forwarding.", flush=True)
        try:
            while True:
                for key, _ in selector.select():
                    try:
                        if key.fileobj is board:
                            data, address = board.recvfrom(65535)
                            # Ignore other devices on the network.
                            if address[0] != args.esp_ip:
                                continue
                            peer = address
                            agent.send(data)
                        else:
                            data = agent.recv(65535)
                            if peer is not None:
                                board.sendto(data, peer)
                    except (ConnectionResetError, ConnectionRefusedError):
                        # An agent restart may generate an ICMP port error.
                        continue
        except KeyboardInterrupt:
            print("\nRelay stopped.")


if __name__ == "__main__":
    main()
