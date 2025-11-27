#!/usr/bin/env python3
import sys
import os
import time
import math
from dataclasses import dataclass

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

# ==========================================
# ==== CONFIGURACIÓN DE PASOS ====
# ==========================================

# DATOS DEL MOTOR Y DRIVER
STEPS_MOTOR = 200.0   # 1.8 grados por paso
MICROSTEPS = 8.0      # Configuración del driver (1/8)

# REDUCCIONES MECÁNICAS
REDUCCION_EJE_1 = 12.0  # 12:1 (Eje principal)
REDUCCION_EJE_2 = 16.0  
REDUCCION_EJE_3 = 16.0  

# CÁLCULO DE RATIOS (Pasos por Grado)
PASOS_POR_GRADO_1 = (STEPS_MOTOR * MICROSTEPS * REDUCCION_EJE_1) / 360.0
PASOS_POR_GRADO_2 = (STEPS_MOTOR * MICROSTEPS * REDUCCION_EJE_2) / 360.0
PASOS_POR_GRADO_3 = (STEPS_MOTOR * MICROSTEPS * REDUCCION_EJE_3) / 360.0

# Ángulo de prueba para P1/P2
ANGULO_PRUEBA = 45.0 

# ==== CONFIGURACIÓN CINEMÁTICA ====
L_BASE_OFFSET = 220.0
L_BRAZO = 270.0
L_ANTEBRAZO = 270.0

LIM_EJE1 = (-180, 180)
LIM_EJE2 = (-90 ,  90)
LIM_EJE3 = (-90 ,  90)

# Definición de Puntos
s1_target = ANGULO_PRUEBA * PASOS_POR_GRADO_1
s2_target = ANGULO_PRUEBA * PASOS_POR_GRADO_2
s3_target = ANGULO_PRUEBA * PASOS_POR_GRADO_3

PRESET_POINTS = [
    [s1_target,  s2_target,  s3_target, f"P1 (Todos a +{ANGULO_PRUEBA:.0f}°)"],
    [-s1_target, -s2_target, -s3_target, f"P2 (Todos a -{ANGULO_PRUEBA:.0f}°)"],
    [s1_target/2, s2_target/2, s3_target/2, "P3 (Mitad +)"],
    [-s1_target/2, -s2_target/2, -s3_target/2, "P4 (Mitad -)"],
    
    # Puntos XYZ (Cinemática Inversa)
    [300, 200, 250, "XYZ: Lateral Derecho"],
    [0, 350, 300,   "XYZ: Frente Centro"],
    [400, 0, 200,   "XYZ: Abajo"],
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
    return sizes

# ==== KINEMATICS ====
class Kinematics:
    def __init__(self): 
        self.theta1, self.theta2, self.theta3 = 0.0, 0.0, 0.0
        self.curr_x, self.curr_y, self.curr_z = self.forward_kinematics_xyz(0.0, 0.0, 0.0)

    def forward_kinematics_xyz(self, d1, d2, d3):
        t1 = math.radians(d1); t2 = math.radians(d2); t3 = math.radians(d3)
        r = L_BRAZO * math.cos(t2) + L_ANTEBRAZO * math.cos(t2 + t3)
        z = L_BASE_OFFSET + L_BRAZO * math.sin(t2) + L_ANTEBRAZO * math.sin(t2 + t3)
        x = r * math.cos(t1)
        y = r * math.sin(t1)
        return x, y, z

    def calculate_ik(self, tx, ty, tz):
        t1_rad = math.atan2(ty, tx)
        r = math.sqrt(tx**2 + ty**2)
        z_prime = tz - L_BASE_OFFSET
        D = math.sqrt(r**2 + z_prime**2)

        if D > (L_BRAZO + L_ANTEBRAZO): return None
        if D < abs(L_BRAZO - L_ANTEBRAZO): return None

        numerator = (r**2 + z_prime**2 - L_BRAZO**2 - L_ANTEBRAZO**2)
        denominator = (2 * L_BRAZO * L_ANTEBRAZO)
        if denominator == 0: return None
        cos_t3 = numerator / denominator
        cos_t3 = max(-1.0, min(1.0, cos_t3))
        t3_rad = math.acos(cos_t3)

        alpha = math.atan2(z_prime, r)
        beta_num = (r**2 + z_prime**2 + L_BRAZO**2 - L_ANTEBRAZO**2)
        beta_den = (2 * D * L_BRAZO)
        if beta_den == 0: return None
        beta_val = max(-1.0, min(1.0, beta_num / beta_den))
        beta = math.acos(beta_val)
        t2_rad = alpha + beta

        d1 = math.degrees(t1_rad)
        d2 = math.degrees(t2_rad)
        d3 = -math.degrees(t3_rad)

        if not (LIM_EJE1[0] <= d1 <= LIM_EJE1[1]): return None
        if not (LIM_EJE2[0] <= d2 <= LIM_EJE2[1]): return None
        if not (LIM_EJE3[0] <= d3 <= LIM_EJE3[1]): return None

        return (d1, d2, d3)

robot = Kinematics()

# -----------------------------------------------------------------
# Hilo lector (Rx)
# -----------------------------------------------------------------
class StatusReader(QtCore.QObject):
    statusReceived = QtCore.pyqtSignal(str)
    def __init__(self):
        super().__init__()
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
    def __init__(self, parent=None, sender_callback=None):
        super().__init__(parent)
        self.setWindowTitle("Modo Manual")
        self.sender = sender_callback
        S = compute_units(self)
        lay = QtWidgets.QVBoxLayout(self)
        
        row = QtWidgets.QHBoxLayout()
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
        if self.sender: self.sender(base, d, 0)

class PresetMoveDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Mover a Punto")
        self.resize(350, 150)
        lay = QtWidgets.QVBoxLayout(self)
        self.combo = QtWidgets.QComboBox()
        for pt in PRESET_POINTS: self.combo.addItem(pt[3])
        lay.addWidget(self.combo)
        btns = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        btns.accepted.connect(self.accept); btns.rejected.connect(self.reject)
        lay.addWidget(btns)
    def get_idx(self): return self.combo.currentIndex()

# ==== VENTANA PRINCIPAL ====
class RobotUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Control Robot - Secuenciador (Soft Reset)")
        
        self.fd_tx = None
        self.is_homing = False
        self.ignore_status_until = 0.0
        self.current_freq = 1000
        
        # --- COLA DE MOVIMIENTOS ---
        self.sequence_queue = []  
        self.is_running_sequence = False

        # UI
        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        lay = QtWidgets.QVBoxLayout(central)

        # Info Ratios
        lbl_info = QtWidgets.QLabel(f"Ratios: E1={PASOS_POR_GRADO_1:.1f}, E2={PASOS_POR_GRADO_2:.1f}, E3={PASOS_POR_GRADO_3:.1f} steps/°")
        lbl_info.setStyleSheet("font-size:10px; color:gray")
        lay.addWidget(lbl_info)

        self.lbl_xyz = QtWidgets.QLabel("XYZ: ...")
        self.lbl_status = QtWidgets.QLabel("IDLE")
        self.lbl_status.setStyleSheet("color:green; font-weight:bold; font-size:16px")
        self.lbl_status.setAlignment(QtCore.Qt.AlignCenter)
        lay.addWidget(self.lbl_xyz); lay.addWidget(self.lbl_status)

        # Slider Velocidad
        sl_lay = QtWidgets.QHBoxLayout()
        self.lbl_vel = QtWidgets.QLabel("1000 Hz")
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(200, 5000); self.slider.setValue(1000) 
        sl_lay.addWidget(QtWidgets.QLabel("Vel:")); sl_lay.addWidget(self.slider); sl_lay.addWidget(self.lbl_vel)
        lay.addLayout(sl_lay)
        
        self.slider.valueChanged.connect(lambda v: self.lbl_vel.setText(f"{v} Hz"))
        self.slider.sliderReleased.connect(self.update_speed)

        # Botones
        grid = QtWidgets.QGridLayout()
        self.b_home = QtWidgets.QPushButton("HOME (Reset 0)")
        self.b_ptp = QtWidgets.QPushButton("PTP Individual")
        self.b_man = QtWidgets.QPushButton("Jog Manual")
        self.b_seq = QtWidgets.QPushButton("▶ EJECUTAR SECUENCIA (P1-P4)")
        self.b_seq.setStyleSheet("background-color: #DDA0DD; font-weight: bold; height: 40px;")

        grid.addWidget(self.b_home, 0, 0)
        grid.addWidget(self.b_ptp, 0, 1)
        grid.addWidget(self.b_man, 1, 0, 1, 2)
        grid.addWidget(self.b_seq, 2, 0, 1, 2)
        lay.addLayout(grid)

        self.log = QtWidgets.QPlainTextEdit(readOnly=True)
        lay.addWidget(self.log)

        # Conexiones
        self.b_home.clicked.connect(self.do_home)
        self.b_ptp.clicked.connect(self.open_ptp)
        self.b_man.clicked.connect(lambda: run_dialog(ManualDialog(self, self.send_spi)))
        self.b_seq.clicked.connect(self.start_sequence)

        self.resize(400, 600)
        
        # Estado inicial Robot
        self.last_a1, self.last_a2, self.last_a3 = 0.0, 0.0, 0.0
        robot.theta1, robot.theta2, robot.theta3 = 0.0, 0.0, 0.0
        self.update_ui()

        self.setup_reader()
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.poll); self.timer.start(1000)
        
        QtCore.QTimer.singleShot(500, self.update_speed)

    # --- LÓGICA DE SECUENCIA ---
    def start_sequence(self):
        if self.is_running_sequence or self.is_homing: return
        
        self.log_cmd("=== INICIANDO SECUENCIA ===")
        points_indices = [0, 1, 2, 3] # Indices de PRESET_POINTS
        self.sequence_queue = []
        
        for idx in points_indices:
            px, py, pz, name = PRESET_POINTS[idx]
            if "P1" in name or "P2" in name or "P3" in name or "P4" in name:
                t1, t2, t3 = px, py, pz
            else:
                res = robot.calculate_ik(px, py, pz)
                if res: 
                    a1, a2, a3 = res
                    t1, t2, t3 = a1, a2, a3 
                else: 
                    self.log_cmd(f"Error IK en {name}"); continue
            
            is_steps = "P" in name.split(" ")[0]
            self.sequence_queue.append((t1, t2, t3, name, is_steps))

        self.is_running_sequence = True
        self.process_next_step()

    def process_next_step(self):
        if not self.sequence_queue:
            self.log_cmd("=== SECUENCIA FINALIZADA ===")
            self.is_running_sequence = False
            self.lbl_status.setText("IDLE")
            self.lbl_status.setStyleSheet("color:green; font-weight:bold; font-size:16px")
            return

        t1, t2, t3, name, is_steps = self.sequence_queue.pop(0)
        self.log_cmd(f"Secuencia: {name}")
        
        if is_steps:
            a1 = t1 / PASOS_POR_GRADO_1
            a2 = t2 / PASOS_POR_GRADO_2
            a3 = t3 / PASOS_POR_GRADO_3
            self.send_kinematic_move(a1, a2, a3)
        else:
            self.send_kinematic_move(t1, t2, t3)

    # --- COMUNICACIÓN ---
    def connect_fifo(self):
        if self.fd_tx: return
        if os.path.exists(FIFO_PATH):
            try: self.fd_tx = os.open(FIFO_PATH, os.O_WRONLY | os.O_NONBLOCK)
            except: pass

    def send_spi(self, cmd, a1, a2):
        self.connect_fifo()
        if self.fd_tx:
            try: os.write(self.fd_tx, f"I {int(cmd)} {int(a1)} {int(a2)}\n".encode())
            except: self.fd_tx = None

    # --- CONTROL ROBOT ---
    def update_speed(self):
        val = self.slider.value()
        self.send_spi(700, val, 0)
        self.log_cmd(f"Velocidad: {val} Hz")

    def do_home(self):
        self.sequence_queue = []
        self.is_running_sequence = False
        self.is_homing = True
        self.ignore_status_until = time.time() + 2.0
        self.send_spi(500, 0, 0)
        
        self.last_a1, self.last_a2, self.last_a3 = 0.0, 0.0, 0.0
        robot.theta1, robot.theta2, robot.theta3 = 0.0, 0.0, 0.0
        self.update_ui()
        self.log_cmd("Homing iniciado. Posición reseteada a 0.")

    def open_ptp(self):
        dlg = PresetMoveDialog(self)
        if run_dialog(dlg) == QtWidgets.QDialog.Accepted:
            idx = dlg.get_idx()
            px, py, pz, name = PRESET_POINTS[idx]
            
            if "P" in name.split(" ")[0]: # Pasos directos (P1, P2...)
                t1 = px / PASOS_POR_GRADO_1
                t2 = py / PASOS_POR_GRADO_2
                t3 = pz / PASOS_POR_GRADO_3
                self.send_kinematic_move(t1, t2, t3)
            else: # Coordenadas XYZ
                res = robot.calculate_ik(px, py, pz)
                if res: self.send_kinematic_move(*res)
                else: self.log_cmd("Fuera de rango")

    def send_kinematic_move(self, a1, a2, a3):
        delta_a1 = a1 - self.last_a1
        delta_a2 = a2 - self.last_a2
        delta_a3 = a3 - self.last_a3
        
        self.last_a1, self.last_a2, self.last_a3 = a1, a2, a3
        robot.theta1, robot.theta2, robot.theta3 = a1, a2, a3
        self.update_ui()

        s1 = int(delta_a1 * PASOS_POR_GRADO_1)
        s2 = int(delta_a2 * PASOS_POR_GRADO_2)
        s3 = int(delta_a3 * PASOS_POR_GRADO_3)

        if s1==0 and s2==0 and s3==0:
            if self.is_running_sequence: QtCore.QTimer.singleShot(100, self.process_next_step)
            return

        self.log_cmd(f"Mover: {s1}, {s2}, {s3} pasos")
        self.send_spi(400, s1, s2)
        time.sleep(0.005)
        self.send_spi(401, s3, 0)
        
        self.lbl_status.setText("MOVIENDO...")
        self.lbl_status.setStyleSheet("color:orange; font-weight:bold; font-size:16px")

    def update_ui(self):
        robot.curr_x, robot.curr_y, robot.curr_z = robot.forward_kinematics_xyz(self.last_a1, self.last_a2, self.last_a3)
        self.lbl_xyz.setText(f"X:{robot.curr_x:.0f} Y:{robot.curr_y:.0f} Z:{robot.curr_z:.0f}")

    def log_cmd(self, t):
        self.log.appendPlainText(f">> {t}")

    def poll(self): self.send_spi(0, 0, 0)

    def setup_reader(self):
        self.th = QtCore.QThread()
        self.reader = StatusReader()
        self.reader.moveToThread(self.th)
        self.th.started.connect(self.reader.run)
        self.reader.statusReceived.connect(self.on_status)
        self.th.start()

    def on_status(self, s):
        if not s or time.time() < self.ignore_status_until: return
        
        # =========================================================
        # ALARMA CRÍTICA CON OPCIÓN DE RESET (SOFT RESET)
        # =========================================================
        if "900" in s:
            self.lbl_status.setText("¡ALARMA CRÍTICA!")
            self.lbl_status.setStyleSheet("background-color: red; color: white; font-weight: bold; font-size: 24px; padding: 10px")
            
            self.is_running_sequence = False
            self.sequence_queue = [] 
            
            # Mostramos cuadro de diálogo con opción de RESET
            msg = QtWidgets.QMessageBox()
            msg.setIcon(QtWidgets.QMessageBox.Critical)
            msg.setWindowTitle("FALLA DE HARDWARE")
            msg.setText("¡DETECCIÓN DE FALLO!\n\nEl firmware ha detectado movimiento sin respuesta de encoders.")
            msg.setInformativeText("Revise que los motores no estén atascados y los cables conectados.")
            
            # Botones personalizados
            btn_reset = msg.addButton("RESET ALARMA (Software)", QtWidgets.QMessageBox.AcceptRole)
            btn_exit = msg.addButton("Ignorar", QtWidgets.QMessageBox.RejectRole)
            
            msg.exec_() # Bloquea hasta que el usuario elija
            
            if msg.clickedButton() == btn_reset:
                self.log_cmd("Enviando comando de RESET (999)...")
                # Enviamos comando 999 repetidas veces para asegurar recepción
                for _ in range(3):
                    self.send_spi(999, 0, 0)
                    time.sleep(0.05)
                
                # Damos un respiro para que el status cambie de 900 a 501
                self.ignore_status_until = time.time() + 1.0
                self.lbl_status.setText("RESET...")
                self.lbl_status.setStyleSheet("color: orange;")
            
            return 

        # =========================================================
        
        if "500" in s:
            txt = "HOMING..." if self.is_homing else "OCUPADO"
            self.lbl_status.setText(txt)
            self.lbl_status.setStyleSheet("color:red; font-weight:bold; font-size:16px" if self.is_homing else "color:orange; font-weight:bold; font-size:16px")
        
        elif "501" in s:
            # Si veníamos de una alarma y ahora es 501, limpiamos el estilo rojo
            if "ALARMA" in self.lbl_status.text() or "RESET" in self.lbl_status.text():
                 self.lbl_status.setText("IDLE")
                 self.lbl_status.setStyleSheet("color:green; font-weight:bold; font-size:16px")

            self.lbl_status.setText("IDLE")
            self.lbl_status.setStyleSheet("color:green; font-weight:bold; font-size:16px")
            
            if self.is_homing:
                self.is_homing = False
                self.log_cmd("Homing Completado")
                QtWidgets.QMessageBox.information(self, "Info", "Homing OK")
            
            elif self.is_running_sequence:
                QtCore.QTimer.singleShot(100, self.process_next_step)

    def closeEvent(self, e):
        self.close_fifo()
        if hasattr(self, 'reader'): self.reader.stop()
        e.accept()

    def close_fifo(self):
        if self.fd_tx: 
            try: os.close(self.fd_tx)
            except: pass
            self.fd_tx = None

def main():
    app = QtWidgets.QApplication(sys.argv)
    win = RobotUI()
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
