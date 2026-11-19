#!/usr/bin/env python3
"""
POWERGuard 5 — Servidor Local HTTP para Web Serial API
Instituto Kriete de Ingeniería y Ciencias · Circuitos Eléctricos
"""

import http.server
import socketserver
import webbrowser
import os
import sys

PORT = 8080

class Handler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        # Desactivar cache para desarrollo y pruebas en caliente
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        super().end_headers()

def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    print("=" * 70)
    print(" POWERGuard 5 — Sistema Automático de Compensación de Factor de Potencia")
    print(" Instituto Kriete de Ingeniería y Ciencias · Circuitos Eléctricos")
    print(" Equipo: Diana Carballo, Emma Palacios, Eduardo Figueroa, Juan Pablo Ramírez")
    print("=" * 70)
    print(f"\n[+] Iniciando servidor HTTP local en: http://localhost:{PORT}")
    print("[+] Acceso completo a Web Serial API (Chrome / Edge / Opera / Brave)")
    print("[+] Presiona Ctrl + C en esta terminal para detener el servidor.\n")

    try:
        with socketserver.TCPServer(("", PORT), Handler) as httpd:
            # Abrir automáticamente en el navegador predeterminado
            webbrowser.open(f"http://localhost:{PORT}")
            httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[*] Servidor detenido por el usuario.")
    except Exception as e:
        print(f"\n[!] Error al iniciar el servidor: {e}")
        print(f"    Puedes intentar abrir index.html directamente en Chrome.")

if __name__ == "__main__":
    main()

