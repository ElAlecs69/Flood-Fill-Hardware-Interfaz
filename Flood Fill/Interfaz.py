"""
Monitor Bluetooth Micromouse con Gyro + Posición
Compatible con firmware flood-fill actual
"""

import serial
from datetime import datetime

PUERTO = "/dev/cu.HC-05"   # Cambia a COMx si estás en Windows
BAUDRATE = 9600


def main():
    print("=" * 90)
    print("   MICROMOUSE MONITOR (Flood Fill + Gyro + Posición)")
    print("=" * 90)

    try:
        ser = serial.Serial(PUERTO, BAUDRATE, timeout=1)
        print(f"✓ Conectado a {PUERTO}\n")
    except Exception as e:
        print(f"✗ Error al conectar: {e}")
        return

    # Encabezado de tabla
    print(
        f"{'Tiempo':<10} "
        f"{'F':>4} {'L':>4} {'R':>4} "
        f"{'Yaw':>7} "
        f"{'X':>3} {'Y':>3} {'Dir':>3}  "
        f"Acción"
    )
    print("-" * 90)

    try:
        while True:
            if ser.in_waiting:
                linea = ser.readline().decode("utf-8", errors="ignore").strip()

                if not linea:
                    continue

                if linea == "START":
                    print("\n>>> MICROMOUSE INICIADO <<<\n")
                    continue

                partes = linea.split(",")

                # Esperamos 8 campos exactos
                if len(partes) != 8:
                    continue

                front, left, right, yaw, rx, ry, rdir, accion = partes

                tiempo = datetime.now().strftime("%H:%M:%S")

                # Advertencia frontal
                warn = "!" if front.isdigit() and int(front) <= 7 else " "

                # Decodificar dirección
                dir_map = {"0": "N", "1": "E", "2": "S", "3": "W"}
                dir_txt = dir_map.get(rdir, "?")

                print(
                    f"{tiempo:<10} "
                    f"{front:>4}{warn} {left:>4} {right:>4} "
                    f"{yaw:>7} "
                    f"{rx:>3} {ry:>3} {dir_txt:>3}  "
                    f"{accion}"
                )

    except KeyboardInterrupt:
        print("\n\nCerrando monitor...")

    finally:
        ser.close()


if __name__ == "__main__":
    main()
