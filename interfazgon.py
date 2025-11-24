import sys
import os
import time
import math
import errno
from dataclasses import dataclass
from typing import Optional

# ==== Qt bindings ====
try:
    from qtpy import QtWidgets, QtCore, QtGui
except Exception:
    try:
        from PySide6 import QtWidgets, QtCore, QtGui
    except Exception:
        try:
            from PyQt6 import QtWidgets, QtCore, QtGui
        except Exception:
            from PyQt5 import QtWidgets, QtCore, QtGui

FIFO_PATH = "/tmp/spi_tx"
FIFO_RX_PATH = "/tmp/spi_rx"

def run_dialog(dlg: QtWidgets.QDialog) -> int:
    return dlg.exec() if hasattr(dlg, "exec") else dlg.exec_()

# ==== CONFIGURACIÓN DEL ROBOT ====
L_BASE_OFFSET = 200.0
L_BRAZO = 250.0
L_ANTEBRAZO = 250.0

# Límites Ángulos (Grados) - Relajados para alcanzar puntos
LIM_EJE1 = (-180, 180)
LIM_EJE2 = (-90 ,  90)
LIM_EJE3 = (-90 ,  90)

# --- HARDWARE (1:16, 8 microsteps) ---
# Total pasos por vuelta = 25,600
# Ratio: ~71.11 pasos por grado
PASOS_POR_GRADO_1 = (200 * 8 * 16) / 360.0
PASOS_POR_GRADO_2 = (200 * 8 * 16) / 360.0
PASOS_POR_GRADO_3 = (200 * 8 * 16) / 360.0

# --- PUNTOS PREDEFINIDOS ---
# P1 y P2 son PASOS ABSOLUTOS (Target Steps) respecto a HOME 0.
# El resto son coordenadas XYZ en mm.
PRESET_POINTS = [
    [2133,  2133,  2133, "P1 (Positivo / +30 grados)"],
    [-2133, -2133, -2133, "P2 (Negativo / -30 grados)"],
    [300,   0, 300, "Punto Seguro (Inicio)"],
    [350,   0, 400, "Punto Alto (Límite)"],
    [400,   0, 200, "Punto Bajo (Recogida)"],
    [300, 200, 250, "Punto Lateral 1"],
    [200, 150, 300, "Punto Cercano"],
    [350, -100, 250, "Punto Intermedio"],
    [480,   0, 250, "Extensión Máxima"],
    [100, 300, 300, "Punto Izquierda"],
    [250,   0, 350, "Punto Retraído Arriba"],
]

def compute_units(widget=None):
    app = QtWidgets.QApplication.instance()
    scr = app.primaryScreen()
    if widget and hasattr(app, "screenAt"):
        s = app.screenAt(widget.mapToGlobal(QtCore.QPoint(0,0)))
        if s: scr = s
    g = scr.availableGeometry()
    dpi = scr.logicalDotsPerInch() if hasattr(scr, "logicalDotsPerInch") else 96.0
    w, h = g.width(), g.height()
    side_min = min(w, h)
    U = max(30, min(int(side_min/13), 100))
    dpi_sf = dpi/96.0 if dpi > 0 else 1.0
    if dpi_sf > 0: U = int(U/dpi_sf)
    sizes = { "U":U, "btn_big":int(U*1.45), "btn_small":int(U*0.95), "pad":int(U*0.26), "radius":int(U*0.22) }
    sizes["btn_big"] = max(54, min(sizes["btn_big"], 120))
    sizes["btn_small"] = max(38, min(sizes["btn_small"], 86))
    sizes["pad"] = max(6, min(sizes["pad"], 16))
    sizes["radius"] = max(6, min(sizes["radius"], 12))
    return sizes

# ==== KINEMATICS ====
class Kinematics:
    def _init_(self):
        x_start, y_start, z_start = 400.0, 0.0, L_BASE_OFFSET + 100.0
        self.curr_x, self.curr_y, self.curr_z = x_start, y_start, z_start
        self.theta1, self.theta2, self.theta3 = 0, 0, 0
        ok, angles = self.update_target(x_start, y_start, z_start)
        if not ok:
            self.theta1, self.theta2, self.theta3 = 0.0, 90.0, -90.0
            self.curr_x, self.curr_y, self.curr_z = 0.0, 0.0, L_BASE_OFFSET + L_ANTEBRAZO

    def forward_kinematics_xyz(self, d1, d2, d3):
        t1 = math.radians(d1); t2 = math.radians(d2); t3 = math.radians(d3)
        r = L_BRAZO * math.cos(t2) + L_ANTEBRAZO * math.cos(t2 + t3)
        z = L_BASE_OFFSET + L_BRAZO * math.sin(t2) + L_ANTEBRAZO * math.sin(t2 + t3)
        x = r * math.cos(t1)
        y = r * math.sin(t1)
        return x, y, z

    def calculate_ik(self, tx, ty, tz):
        t1_rad = math.atan2(ty, tx)
        r = math.sqrt(tx*2 + ty*2)
        z_prime = tz - L_BASE_OFFSET
        D = math.sqrt(r*2 + z_prime*2)

        if D > (L_BRAZO + L_ANTEBRAZO):
            print(f"[IK ERROR] Punto fuera de alcance: Distancia {D:.1f} > Max {L_BRAZO+L_ANTEBRAZO}")
            return None
        if D < abs(L_BRAZO - L_ANTEBRAZO):
            print("[IK ERROR] Punto demasiado cerca del eje")
            return None

        cos_t3 = (r*2 + z_prime2 - L_BRAZO2 - L_ANTEBRAZO*2) / (2 * L_BRAZO * L_ANTEBRAZO)
        cos_t3 = max(-1.0, min(1.0, cos_t3))
        t3_rad = math.acos(cos_t3)

        alpha = math.atan2(z_prime, r)
        beta = math.acos((r*2 + z_prime2 + L_BRAZO2 - L_ANTEBRAZO*2) / (2 * D * L_BRAZO))
        t2_rad = alpha + beta

        d1 = math.degrees(t1_rad)
        d2 = math.degrees(t2_rad)
        d3 = -math.degrees(t3_rad)

        if not (LIM_EJE1[0] <= d1 <= LIM_EJE1[1]):
            print(f"[IK ERROR] Eje 1 fuera de limite: {d1:.1f}°")
            return None
        if not (LIM_EJE2[0] <= d2 <= LIM_EJE2[1]):
            print(f"[IK ERROR] Eje 2 fuera de limite: {d2:.1f}°")
            return None
        if not (LIM_EJE3[0] <= d3 <= LIM_EJE3[1]):
            print(f"[IK ERROR] Eje 3 fuera de limite: {d3:.1f}°")
            return None

        return (d1, d2, d3)

    def update_target(self, x, y, z):
        res = self.calculate_ik(x, y, z)
        if res:
            self.curr_x, self.curr_y, self.curr_z = x, y, z
            self.theta1, self.theta2, self.theta3 = res
            return True, res
        return False, None

robot = Kinematics()

# -----------------------------------------------------------------
# Hilo lector (Rx)
# -----------------------------------------------------------------
class StatusReader(QtCore.QObject):
    statusReceived = QtCore.pyqtSignal(str)
    def _init_(self):
        super()._init_()
        self.is_running = True
    def run(self):
        while self.is_running:
            try:
                if not os.path.exists(FIFO_RX_PATH):
                    time.sleep(1); continue
                with open(FIFO_RX_PATH, 'r') as fifo:
                    while self.is_running:
                        line = fifo.readline()
                        if line: self.statusReceived.emit(line.strip())
                        else: time.sleep(1); break
            except Exception: time.sleep(1)
    def stop(self): self.is_running = False

# ==== DIÁLOGOS ====
class ManualDialog(QtWidgets.QDialog):
    def _init_(self, parent=None, sender_callback=None):
        super()._init_(parent)
        self.setWindowTitle("Modo Manual")
        self.sender = sender_callback
        S = compute_units(self)
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(S["pad"],S["pad"],S["pad"],S["pad"])

        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Eje:"))
        self.combo = QtWidgets.QComboBox()
        self.combo.addItems(["Eje 1","Eje 2","Eje 3"])
        row.addWidget(self.combo); lay.addLayout(row)

        btns = QtWidgets.QHBoxLayout()
        self.b_ccw = QtWidgets.QPushButton("◀ (-)"); btns.addWidget(self.b_ccw)
        self.b_stop = QtWidgets.QPushButton("■ STOP"); btns.addWidget(self.b_stop)
        self.b_cw = QtWidgets.QPushButton("▶ (+)"); btns.addWidget(self.b_cw)
        lay.addLayout(btns)

        self.lbl = QtWidgets.QLabel("Listo")
        self.lbl.setAlignment(QtCore.Qt.AlignCenter); lay.addWidget(self.lbl)

        self.b_ccw.pressed.connect(lambda: self.mv(2))
        self.b_ccw.released.connect(lambda: self.mv(0))
        self.b_cw.pressed.connect(lambda: self.mv(1))
        self.b_cw.released.connect(lambda: self.mv(0))
        self.b_stop.clicked.connect(lambda: self.mv(0))

    def mv(self, d):
        base = 601 + self.combo.currentIndex()
        txt = "STOP"
        if d==1: txt="CW (+)"
        elif d==2: txt="CCW (-)"
        self.lbl.setText(f"Eje {self.combo.currentIndex()+1}: {txt}")
        if self.sender: self.sender(base, d, 0)

class PresetMoveDialog(QtWidgets.QDialog):
    def _init_(self, parent=None, sender_callback=None):
        super()._init_(parent)
        self.setWindowTitle("Mover a Punto Predefinido")
        self.resize(300, 150)
        lay = QtWidgets.QVBoxLayout(self)
        lay.addWidget(QtWidgets.QLabel("Selecciona un punto objetivo:"))
        self.combo_points = QtWidgets.QComboBox()
        for pt in PRESET_POINTS:
            self.combo_points.addItem(f"{pt[3]} ({pt[0]}, {pt[1]}, {pt[2]})")
        lay.addWidget(self.combo_points)
        btns = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        lay.addWidget(btns)
    def get_selected_point_index(self):
        return self.combo_points.currentIndex()

# ==== VENTANA PRINCIPAL ====
class RobotUI(QtWidgets.QMainWindow):
    def _init_(self):
        super()._init_()
        self.setWindowTitle("Control Robot - PTP/MoveJ")
        self.S = compute_units(self)

        self.fd_tx = None 
        self.is_homing = False

        # UI
        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        lay = QtWidgets.QVBoxLayout(central)

        self.lbl_xyz = QtWidgets.QLabel("XYZ: ...")
        self.lbl_ang = QtWidgets.QLabel("Ang: ...")
        self.lbl_status = QtWidgets.QLabel("IDLE")
        self.lbl_xyz.setStyleSheet("font-size:14px; font-weight:bold; color:blue;")
        lay.addWidget(self.lbl_xyz); lay.addWidget(self.lbl_ang); lay.addWidget(self.lbl_status)

        grid = QtWidgets.QGridLayout()
        self.b_up = QtWidgets.QPushButton("▲ E2+")
        self.b_dn = QtWidgets.QPushButton("▼ E2-")
        self.b_lt = QtWidgets.QPushButton("◀ E1-")
        self.b_rt = QtWidgets.QPushButton("▶ E1+")
        self.b_home = QtWidgets.QPushButton("HOME")
        self.b_man = QtWidgets.QPushButton("Manual")
        self.b_ptp = QtWidgets.QPushButton("MOVE PTP")

        grid.addWidget(self.b_up, 0, 1)
        grid.addWidget(self.b_lt, 1, 0)
        grid.addWidget(self.b_home, 1, 1)
        grid.addWidget(self.b_rt, 1, 2)
        grid.addWidget(self.b_dn, 2, 1)
        grid.addWidget(self.b_man, 3, 0, 1, 3)
        grid.addWidget(self.b_ptp, 4, 0, 1, 3)
        lay.addLayout(grid)

        self.log = QtWidgets.QPlainTextEdit(readOnly=True)
        lay.addWidget(self.log)

        # Conexiones
        self.b_up.clicked.connect(lambda: self.move_joint(2, 5))
        self.b_dn.clicked.connect(lambda: self.move_joint(2, -5))
        self.b_lt.clicked.connect(lambda: self.move_joint(1, -5))
        self.b_rt.clicked.connect(lambda: self.move_joint(1, 5))
        self.b_home.clicked.connect(self.do_home)
        self.b_man.clicked.connect(self._open_manual_dialog)
        self.b_ptp.clicked.connect(self._open_preset_dialog)

        self.resize(400, 500)

        self.last_a1 = robot.theta1
        self.last_a2 = robot.theta2
        self.last_a3 = robot.theta3
        self.update_labels()

        self.setup_status_reader_thread()
        self.poll_timer = QtCore.QTimer(self)
        self.poll_timer.timeout.connect(self.poll_status)
        self.poll_timer.start(1000)

    # --- CONEXIÓN FIFO ROBUSTA ---
    def connect_fifo(self):
        if self.fd_tx is not None: return
        if not os.path.exists(FIFO_PATH): return
        try:
            self.fd_tx = os.open(FIFO_PATH, os.O_WRONLY | os.O_NONBLOCK)
            self.log_cmd("FIFO TX conectado.")
        except OSError as e:
            if e.errno != errno.ENXIO: self.log_cmd(f"FIFO Err: {e}")

    def close_fifo(self):
        if self.fd_tx is not None:
            try: os.close(self.fd_tx)
            except Exception: pass
            self.fd_tx = None

    def send_spi(self, cmd, a1, a2):
        if self.fd_tx is None:
            self.connect_fifo()
            if self.fd_tx is None: return

        msg = f"I {int(cmd)} {int(a1)} {int(a2)}\n".encode()
        try:
            os.write(self.fd_tx, msg)
            if cmd != 0: self.log_cmd(f"TX: {cmd} {a1} {a2}")
        except OSError as e:
            if e.errno == errno.EPIPE: self.close_fifo()
            elif e.errno != errno.EAGAIN: self.log_cmd(f"Err TX: {e}"); self.close_fifo()

    # -------------------------------------------

    def move_joint(self, axis, delta):
        if self.is_homing: return
        a1, a2, a3 = robot.theta1, robot.theta2, robot.theta3

        if axis == 1: a1 += delta
        elif axis == 2: a2 += delta
        elif axis == 3: a3 += delta

        a1 = max(LIM_EJE1[0], min(LIM_EJE1[1], a1))
        a2 = max(LIM_EJE2[0], min(LIM_EJE2[1], a2))
        a3 = max(LIM_EJE3[0], min(LIM_EJE3[1], a3))

        x, y, z = robot.forward_kinematics_xyz(a1, a2, a3)

        ok, angs = robot.update_target(x, y, z)
        if ok:
            self.update_labels()
            self.send_kinematic_move(*angs)
        else:
            self.log_cmd("Límite alcanzado (ver terminal)")

    def _open_preset_dialog(self):
        if self.is_homing: return
        dlg = PresetMoveDialog(self)
        if run_dialog(dlg) == QtWidgets.QDialog.Accepted:
            idx = dlg.get_selected_point_index()
            px, py, pz, name = PRESET_POINTS[idx]

            # === LÓGICA ESPECIAL P1 / P2 (Pasos Directos) ===
            if "P1" in name or "P2" in name:
                self.log_cmd(f"PTP Directo (Pasos): {name}")

                t1 = px / PASOS_POR_GRADO_1
                t2 = py / PASOS_POR_GRADO_2
                t3 = pz / PASOS_POR_GRADO_3

                robot.theta1, robot.theta2, robot.theta3 = t1, t2, t3
                robot.curr_x, robot.curr_y, robot.curr_z = robot.forward_kinematics_xyz(t1, t2, t3)

                self.update_labels()
                self.send_kinematic_move(t1, t2, t3)

            # === LÓGICA NORMAL (Coordenadas XYZ) ===
            else:
                self.log_cmd(f"PTP Objetivo: {name}")
                ok, angs = robot.update_target(px, py, pz)
                if ok:
                    self.update_labels()
                    self.send_kinematic_move(*angs)
                else:
                    self.log_cmd("Error: Punto fuera de rango (ver terminal)")

    def update_labels(self):
        self.lbl_xyz.setText(f"X:{robot.curr_x:.0f} Y:{robot.curr_y:.0f} Z:{robot.curr_z:.0f}")
        self.lbl_ang.setText(f"A1:{robot.theta1:.1f} A2:{robot.theta2:.1f} A3:{robot.theta3:.1f}")

    def send_kinematic_move(self, a1, a2, a3):
        s1 = int((a1 - self.last_a1) * PASOS_POR_GRADO_1)
        s2 = int((a2 - self.last_a2) * PASOS_POR_GRADO_2)
        s3 = int((a3 - self.last_a3) * PASOS_POR_GRADO_3)

        self.last_a1, self.last_a2, self.last_a3 = a1, a2, a3

        if s1==0 and s2==0 and s3==0: return

        self.send_spi(400, s1, s2)
        time.sleep(0.002)
        self.send_spi(401, s3, 0)

    def do_home(self):
        self.send_spi(500, 0, 0)
        robot.update_target(400, 0, 300)
        self.last_a1, self.last_a2, self.last_a3 = robot.theta1, robot.theta2, robot.theta3
        self.update_labels()

    def _open_manual_dialog(self):
        run_dialog(ManualDialog(self, self.send_spi))

    def log_cmd(self, t):
        self.log.appendPlainText(f">> {t}")
        c = self.log.textCursor(); c.movePosition(QtGui.QTextCursor.End); self.log.setTextCursor(c)

    # --- CORRECCIÓN CLAVE DEL BUG DE IDLE ---
    def poll_status(self):
        # Siempre preguntar, incluso en Homing, para detectar el cambio de 500 -> 501
        self.send_spi(0, 0, 0)

    def setup_status_reader_thread(self):
        self.r_thread = QtCore.QThread()
        self.reader = StatusReader()
        self.reader.moveToThread(self.r_thread)
        self.r_thread.started.connect(self.reader.run)
        self.reader.statusReceived.connect(self.handle_status)
        self.r_thread.start()

    def handle_status(self, s):
        if not s: return
        if "500" in s:
            self.is_homing = True
            self.lbl_status.setText("HOMING...")
            self.lbl_status.setStyleSheet("color:red")
        elif "501" in s:
            if self.is_homing:
                self.is_homing = False
                self.lbl_status.setText("IDLE")
                self.lbl_status.setStyleSheet("color:green")
                QtWidgets.QMessageBox.information(self, "Info", "Homing OK")

    def closeEvent(self, e):
        self.close_fifo()
        if hasattr(self, 'reader'): self.reader.stop()
        e.accept()

def main():
    app = QtWidgets.QApplication(sys.argv)
    win = RobotUI()
    win.show()
    sys.exit(app.exec_())

if _name_ == "_main_":
    main()
