#!/usr/bin/env python3
"""Temporary HTTP/HTTPS proxy for the private G1 Ethernet link."""

import select
import socket
import socketserver
from urllib.parse import urlsplit


LISTEN_HOST = "192.168.123.100"
LISTEN_PORT = 8899
BUFFER_SIZE = 65536


def connect_ipv4(host, port):
    address = socket.getaddrinfo(
        host, port, socket.AF_INET, socket.SOCK_STREAM
    )[0][-1]
    upstream = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    upstream.settimeout(20)
    upstream.connect(address)
    return upstream


class ProxyHandler(socketserver.BaseRequestHandler):
    def handle(self):
        request = self._read_headers()
        if not request:
            return

        request_line, separator, remainder = request.partition(b"\r\n")
        parts = request_line.decode("iso-8859-1").split()
        if len(parts) != 3:
            return

        method, target, _ = parts
        if method.upper() == "CONNECT":
            host, port = target.rsplit(":", 1)
            upstream = connect_ipv4(host, int(port))
            self.request.sendall(b"HTTP/1.1 200 Connection established\r\n\r\n")
            self._relay(upstream)
            return

        destination = urlsplit(target)
        host = destination.hostname
        if not host:
            return
        port = destination.port or 80
        path = destination.path or "/"
        if destination.query:
            path += f"?{destination.query}"
        upstream = connect_ipv4(host, port)
        upstream.sendall(f"{method} {path} HTTP/1.1\r\n".encode())
        upstream.sendall(
            remainder.replace(b"Proxy-Connection:", b"Connection:")
        )
        self._relay(upstream)

    def _read_headers(self):
        data = b""
        while b"\r\n\r\n" not in data and len(data) < 65536:
            chunk = self.request.recv(BUFFER_SIZE)
            if not chunk:
                return b""
            data += chunk
        return data

    def _relay(self, upstream):
        try:
            sockets = [self.request, upstream]
            while True:
                readable, _, _ = select.select(sockets, [], [], 60)
                if not readable:
                    return
                for source in readable:
                    data = source.recv(BUFFER_SIZE)
                    if not data:
                        return
                    (upstream if source is self.request else self.request).sendall(data)
        finally:
            upstream.close()


class ThreadingProxy(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


if __name__ == "__main__":
    with ThreadingProxy((LISTEN_HOST, LISTEN_PORT), ProxyHandler) as server:
        server.serve_forever()