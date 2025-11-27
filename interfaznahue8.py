#!/usr/bin/env python3
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

# ==========================================
# ==== CONFIGURACIÓN DEL ROBOT ====
# ==========================================

# DATOS DEL MOTOR Y DRIVER
STEPS_MOTOR = 200.0   # 1.8 grados por paso
MICROSTEPS = 8.0      # Configuración del driver (1/8)

# REDUCCIONES MECÁNICAS
REDUCCION_EJE_1 = 16.0  
REDUCCION_EJE_2 = 16.0  
REDUCCION_EJE_3 = 16.0  

# CÁLCULO DE RATIOS (Pasos por Grado)
PASOS_POR_GRADO_1 = (STEPS_MOTOR * MICROSTEPS * REDUCCION_EJE_1) / 360.0
PASOS_POR_GRADO_2 = (STEPS_MOTOR * MICROSTEPS * REDUCCION_EJE_2) / 360.0
PASOS_POR_GRADO_3 = (STEPS_MOTOR * MICROSTEPS * REDUCCION_EJE_3) / 360.0

# ==== CONFIGURACIÓN CINEMÁTICA ====
L_BASE_OFFSET = 200.0
L_BRAZO = 250.0
L_ANTEBRAZO = 250.0

LIM_EJE1 = (-180, 180)
LIM_EJE2 = (-100, 135)
LIM_EJE3 = (-135, 135)

# Puntos Predefinidos
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
        
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(20, 20, 20, 20)
        
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Eje a mover:"))
        self.combo = QtWidgets.QComboBox()
        self.combo.addItems(["Eje 1 (Base)", "Eje 2 (Brazo)", "Eje 3 (Antebrazo)"])
        row.addWidget(self.combo)
        lay.addLayout(row)
        
        btns = QtWidgets.QHBoxLayout()
        
        style_btn = "font-size: 18px; font-weight: bold; padding: 10px; border-radius: 8px;"
        self.b_ccw = QtWidgets.QPushButton("◀ (Negativo)")
        self.b_ccw.setStyleSheet(style_btn + "background-color: #FF9800; color: white;")
        
        self.b_stop = QtWidgets.QPushButton("■ STOP")
        self.b_stop.setStyleSheet(style_btn + "background-color: #D32F2F; color: white;")
        
        self.b_cw = QtWidgets.QPushButton("▶ (Positivo)")
        self.b_cw.setStyleSheet(style_btn + "background-color: #FF9800; color: white;")
        
        btns.addWidget(self.b_ccw)
        btns.addWidget(self.b_stop)
        btns.addWidget(self.b_cw)
        lay.addLayout(btns)
        
        self.lbl = QtWidgets.QLabel("Mantén presionado para mover")
        self.lbl.setAlignment(QtCore.Qt.AlignCenter)
        lay.addWidget(self.lbl)
        
        self.b_ccw.pressed.connect(lambda: self.mv(2))
        self.b_ccw.released.connect(lambda: self.mv(0))
        self.b_cw.pressed.connect(lambda: self.mv(1))
        self.b_cw.released.connect(lambda: self.mv(0))
        self.b_stop.clicked.connect(lambda: self.mv(0))
        
    def mv(self, d):
        base = 601 + self.combo.currentIndex()
        txt = "STOP"
        if d==1: txt="MOVIENDO POSITIVO (+)"
        elif d==2: txt="MOVIENDO NEGATIVO (-)"
        self.lbl.setText(txt)
        if self.sender: self.sender(base, d, 0)

class PresetMoveDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Mover a Punto Predefinido")
        self.resize(350, 150)
        lay = QtWidgets.QVBoxLayout(self)
        lay.addWidget(QtWidgets.QLabel("Selecciona un punto objetivo:"))
        self.combo_points = QtWidgets.QComboBox()
        for pt in PRESET_POINTS:
            self.combo_points.addItem(f"{pt[3]}")
        lay.addWidget(self.combo_points)
        btns = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        lay.addWidget(btns)
    def get_selected_point_index(self):
        return self.combo_points.currentIndex()

# ==== VENTANA PRINCIPAL CON ESTILO MEJORADO ====
class RobotUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Control Robot - Interfaz Tesis")
        self.setMinimumSize(900, 550)
        
        # Variables Lógicas
        self.fd_tx = None
        self.is_homing = False
        self.alarm_active = False
        self.ignore_status_until = 0
        self.sequence_queue = []  
        self.is_running_sequence = False
        self.is_stopped = False

        # Estado inicial Robot
        self.last_a1, self.last_a2, self.last_a3 = 0.0, 0.0, 0.0
        
        # === UI LAYOUT ===
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QGridLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)

        # --- COLUMNA 1: CONTROL DIRECCIONAL (PAD) ---
        self.btn_up    = self._make_dir_button("▲ Eje 2+", "Arriba", orange=True)
        self.btn_down  = self._make_dir_button("▼ Eje 2-", "Abajo",  orange=True)
        self.btn_left  = self._make_dir_button("◀ Eje 1-", "Izquierda", orange=True)
        self.btn_right = self._make_dir_button("▶ Eje 1+", "Derecha",   orange=True)
        self.btn_stop_play = self._make_dir_button("■", "Stop / Play", orange=False)
        self._apply_stop_style()

        pad = QtWidgets.QGridLayout()
        pad.addWidget(self.btn_up,        0, 1)
        pad.addWidget(self.btn_left,      1, 0)
        pad.addWidget(self.btn_stop_play, 1, 1)  
        pad.addWidget(self.btn_right,     1, 2)
        pad.addWidget(self.btn_down,      2, 1)
        
        pad_box = self._group("Control Manual Rápido", pad)
        root.addWidget(pad_box, 0, 0, 2, 1)

        # --- COLUMNA 2: ACCIONES Y ESTADO ---
        col_center = QtWidgets.QVBoxLayout()
        
        # Status Box
        self.lbl_status = QtWidgets.QLabel("IDLE")
        self.lbl_status.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_status.setStyleSheet("font-size: 24px; font-weight: bold; color: white; background-color: green; border-radius: 5px; padding: 10px;")
        col_center.addWidget(self.lbl_status)

        # Info XYZ y Angulos
        self.lbl_xyz = QtWidgets.QLabel("XYZ: 0, 0, 0")
        self.lbl_ang = QtWidgets.QLabel("ANG: 0°, 0°, 0°")
        self.lbl_xyz.setStyleSheet("font-size: 14px; font-weight: bold;")
        self.lbl_ang.setStyleSheet("font-size: 12px; color: gray;")
        
        info_box_layout = QtWidgets.QVBoxLayout()
        info_box_layout.addWidget(self.lbl_xyz)
        info_box_layout.addWidget(self.lbl_ang)
        col_center.addWidget(self._group("Posición Actual", info_box_layout))

        # --- SLIDER VELOCIDAD (INTEGRADO) ---
        slider_layout = QtWidgets.QVBoxLayout()
        
        self.lbl_vel_val = QtWidgets.QLabel("1000 Hz")
        self.lbl_vel_val.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_vel_val.setStyleSheet("font-weight: bold; color: #555;")
        
        self.slider_speed = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_speed.setRange(200, 5000)
        self.slider_speed.setValue(1000)
        
        slider_layout.addWidget(self.slider_speed)
        slider_layout.addWidget(self.lbl_vel_val)
        
        col_center.addWidget(self._group("Ajuste Velocidad Global", slider_layout))
        # ---------------------------------------

        # Botones Circulares y Acciones
        btn_layout = QtWidgets.QVBoxLayout()
        
        btn_row = QtWidgets.QHBoxLayout()
        self.btn_manual = self._make_round_button("JOG", "Modo Manual (Eje por Eje)")
        self.btn_ptp    = self._make_round_button("PTP", "Mover a Punto (Lista)")
        btn_row.addWidget(self.btn_manual)
        btn_row.addWidget(self.btn_ptp)
        
        self.btn_seq    = QtWidgets.QPushButton("▶ EJECUTAR SECUENCIA")
        self.btn_seq.setMinimumHeight(45)
        self.btn_seq.setStyleSheet("background-color: #9C27B0; color: white; font-weight: bold; font-size: 14px; border-radius: 10px;")

        btn_layout.addLayout(btn_row)
        btn_layout.addWidget(self.btn_seq)
        
        col_center.addWidget(self._group("Modos de Operación", btn_layout))
        
        self.btn_home = QtWidgets.QPushButton("🏠 HOME ALL")
        self.btn_home.setMinimumHeight(45)
        self.btn_home.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; font-size: 14px; border-radius: 10px;")
        col_center.addWidget(self.btn_home)

        root.addLayout(col_center, 0, 1, 2, 1)

        # --- FILA INFERIOR: LOG ---
        self.log = QtWidgets.QPlainTextEdit(readOnly=True)
        self.log.setMaximumHeight(120)
        self.log.setPlaceholderText("Historial de comunicación...")
        
        log_box = self._group("Registro de Comandos", QtWidgets.QVBoxLayout())
        log_box.layout().addWidget(self.log)
        root.addWidget(log_box, 2, 0, 1, 2)

        # --- CONEXIONES ---
        self._wire_buttons()

        # --- INICIO ---
        self.setup_reader()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.poll_status)
        self.timer.start(1000)
        
        self.update_ui()
        
    # ---------- Helpers UI ----------
    def _group(self, title, inner_layout):
        box = QtWidgets.QGroupBox(title)
        if isinstance(inner_layout, QtWidgets.QLayout): box.setLayout(inner_layout)
        return box

    def _make_dir_button(self, text, tooltip, orange=False):
        btn = QtWidgets.QPushButton(text)
        btn.setToolTip(tooltip)
        btn.setMinimumSize(80, 80)
        font = btn.font(); font.setPointSize(14); font.setBold(True); btn.setFont(font)
        
        color = "#FF9800" if orange else "#F44336" 
        hover = "#F57C00" if orange else "#D32F2F"
        
        btn.setStyleSheet(f"""
            QPushButton {{ background-color: {color}; color: white; border-radius: 10px; border: 2px solid #333; }}
            QPushButton:hover {{ background-color: {hover}; }}
            QPushButton:pressed {{ background-color: #333; }}
        """)
        return btn

    def _make_round_button(self, text, tooltip):
        btn = QtWidgets.QPushButton(text)
        btn.setToolTip(tooltip)
        btn.setFixedSize(70, 70)
        btn.setStyleSheet("""
            QPushButton { background-color: #607D8B; color: white; border-radius: 35px; font-weight: bold; border: 2px solid #455A64; }
            QPushButton:hover { background-color: #78909C; }
            QPushButton:pressed { background-color: #37474F; }
        """)
        return btn

    def _apply_stop_style(self):
        if self.is_stopped:
            self.btn_stop_play.setText("▶")
            self.btn_stop_play.setStyleSheet("background-color: #4CAF50; color: white; border-radius: 10px; font-size: 24px;")
        else:
            self.btn_stop_play.setText("■")
            self.btn_stop_play.setStyleSheet("background-color: #F44336; color: white; border-radius: 10px; font-size: 24px;")

    # ---------- Lógica de Control ----------
    def _wire_buttons(self):
        self.btn_up.clicked.connect(lambda: self.move_joint(2, 5))
        self.btn_down.clicked.connect(lambda: self.move_joint(2, -5))
        self.btn_left.clicked.connect(lambda: self.move_joint(1, -5))
        self.btn_right.clicked.connect(lambda: self.move_joint(1, 5))
        
        self.btn_home.clicked.connect(self.do_home)
        self.btn_manual.clicked.connect(self._open_manual_dialog)
        self.btn_ptp.clicked.connect(self._open_preset_dialog)
        self.btn_seq.clicked.connect(self.start_sequence)
        self.btn_stop_play.clicked.connect(self._toggle_stop)
        
        # Slider Conexión
        self.slider_speed.valueChanged.connect(lambda v: self.lbl_vel_val.setText(f"{v} Hz"))
        # AL SOLTAR EL SLIDER -> Enviar CMD 700
        self.slider_speed.sliderReleased.connect(self.update_speed)

    def _toggle_stop(self):
        self.is_stopped = not self.is_stopped
        self._apply_stop_style()
        # Aquí puedes agregar lógica de STOP real si el ESP lo soporta (ej: CMD 600)
        if self.is_stopped:
             self.log_cmd("PAUSA (Stop)")

    def update_speed(self):
        val = self.slider_speed.value()
        self.log_cmd(f"Set Velocidad: {val} Hz")
        # Enviamos CMD 700 (Configurar Velocidad)
        # Arg1 = Frecuencia, Arg2 = 0
        self.send_spi(700, val, 0)

    # --- Lógica de Secuencia ---
    def start_sequence(self):
        if self.is_running_sequence or self.is_homing: return
        self.log_cmd("=== INICIANDO SECUENCIA ===")
        p_indices = [0, 1] 
        self.sequence_queue = []
        for idx in p_indices:
            px, py, pz, name = PRESET_POINTS[idx]
            self.sequence_queue.append((px, py, pz, name))
        self.is_running_sequence = True
        self.process_next_step()

    def process_next_step(self):
        if not self.sequence_queue:
            self.log_cmd("=== SECUENCIA FINALIZADA ===")
            self.is_running_sequence = False
            self.lbl_status.setText("IDLE")
            self.lbl_status.setStyleSheet("background-color: green; color: white; font-weight: bold; padding: 10px;")
            return

        px, py, pz, name = self.sequence_queue.pop(0)
        self.log_cmd(f"Secuencia: {name}")
        
        if "P1" in name or "P2" in name:
            t1 = px / PASOS_POR_GRADO_1
            t2 = py / PASOS_POR_GRADO_2
            t3 = pz / PASOS_POR_GRADO_3
            robot.theta1, robot.theta2, robot.theta3 = t1, t2, t3
            robot.curr_x, robot.curr_y, robot.curr_z = robot.forward_kinematics_xyz(t1, t2, t3)
            self.update_ui()
            self.send_kinematic_move(t1, t2, t3)
        else:
            self.log_cmd(f"PTP Objetivo: {name}")
            ok, angs = robot.update_target(px, py, pz)
            if ok:
                self.update_ui()
                self.send_kinematic_move(*angs)

    def _open_preset_dialog(self):
        if self.is_homing: return
        dlg = PresetMoveDialog(self)
        if run_dialog(dlg) == QtWidgets.QDialog.Accepted:
            idx = dlg.get_selected_point_index()
            px, py, pz, name = PRESET_POINTS[idx]
            
            if "P1" in name or "P2" in name:
                self.log_cmd(f"PTP Directo: {name}")
                t1 = px / PASOS_POR_GRADO_1
                t2 = py / PASOS_POR_GRADO_2
                t3 = pz / PASOS_POR_GRADO_3
                robot.theta1, robot.theta2, robot.theta3 = t1, t2, t3
                robot.curr_x, robot.curr_y, robot.curr_z = robot.forward_kinematics_xyz(t1, t2, t3)
                self.update_ui()
                self.send_kinematic_move(t1, t2, t3)
            else:
                self.log_cmd(f"PTP Objetivo: {name}")
                ok, angs = robot.update_target(px, py, pz)
                if ok:
                    self.update_ui()
                    self.send_kinematic_move(*angs)
                else:
                    self.log_cmd("Error: Punto fuera de rango")

    def _open_manual_dialog(self):
        run_dialog(ManualDialog(self, self.send_spi))

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
            self.update_ui()
            self.send_kinematic_move(*angs)
        else:
            self.log_cmd("Límite alcanzado")

    def send_kinematic_move(self, a1, a2, a3):
        s1 = int((a1 - self.last_a1) * PASOS_POR_GRADO_1)
        s2 = int((a2 - self.last_a2) * PASOS_POR_GRADO_2)
        s3 = int((a3 - self.last_a3) * PASOS_POR_GRADO_3)
        self.last_a1, self.last_a2, self.last_a3 = a1, a2, a3
        
        if s1==0 and s2==0 and s3==0:
             if self.is_running_sequence: QtCore.QTimer.singleShot(100, self.process_next_step)
             return
        
        self.send_spi(400, s1, s2)
        time.sleep(0.002) 
        self.send_spi(401, s3, 0)
        self.lbl_status.setText("MOVIENDO...")
        self.lbl_status.setStyleSheet("background-color: orange; color: white; font-weight: bold; padding: 10px;")

    def do_home(self):
        self.send_spi(500, 0, 0)
        robot.update_target(400, 0, 300) 
        self.last_a1, self.last_a2, self.last_a3 = robot.theta1, robot.theta2, robot.theta3
        self.update_ui()

    def update_ui(self):
        self.lbl_xyz.setText(f"XYZ: {robot.curr_x:.0f}, {robot.curr_y:.0f}, {robot.curr_z:.0f}")
        self.lbl_ang.setText(f"Ang: {robot.theta1:.1f}°, {robot.theta2:.1f}°, {robot.theta3:.1f}°")

    # --- COMUNICACIÓN ---
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

    def log_cmd(self, t):
        self.log.appendPlainText(f">> {t}")
        c = self.log.textCursor(); c.movePosition(QtGui.QTextCursor.End); self.log.setTextCursor(c)

    def poll_status(self):
        if not self.alarm_active:
            self.send_spi(0, 0, 0)

    def setup_reader(self):
        self.th = QtCore.QThread()
        self.reader = StatusReader()
        self.reader.moveToThread(self.th)
        self.th.started.connect(self.reader.run)
        self.reader.statusReceived.connect(self.on_status)
        self.th.start()

    def on_status(self, s):
        if self.alarm_active: return
        if not s: return
        
        # ALARMA
        if "900" in s:
            self.alarm_active = True
            self.is_running_sequence = False
            self.sequence_queue = []
            self.lbl_status.setText("¡ALARMA CRÍTICA!")
            self.lbl_status.setStyleSheet("background-color: red; color: white; font-weight: bold; padding: 10px;")
            
            msg = QtWidgets.QMessageBox(self)
            msg.setIcon(QtWidgets.QMessageBox.Critical)
            msg.setWindowTitle("FALLA DE HARDWARE")
            msg.setText("¡ERROR DE ENCODERS!\nEl robot se mueve pero no detecta posición.")
            msg.setStandardButtons(QtWidgets.QMessageBox.Ok)
            if msg.exec() == QtWidgets.QMessageBox.Ok:
                self.alarm_active = False
                self.lbl_status.setText("IDLE")
                self.lbl_status.setStyleSheet("background-color: green; color: white; font-weight: bold; padding: 10px;")
            return

        # NORMAL
        if "500" in s:
            self.is_homing = True
            self.lbl_status.setText("HOMING...")
            self.lbl_status.setStyleSheet("background-color: orange; color: white; font-weight: bold; padding: 10px;")
        elif "501" in s:
            if self.is_homing:
                self.is_homing = False
                self.log_cmd("Homing Finalizado")
                QtWidgets.QMessageBox.information(self, "Info", "Homing OK")
            
            if self.lbl_status.text() != "IDLE":
                self.lbl_status.setText("IDLE")
                self.lbl_status.setStyleSheet("background-color: green; color: white; font-weight: bold; padding: 10px;")
                
            if self.is_running_sequence:
                QtCore.QTimer.singleShot(100, self.process_next_step)

    def closeEvent(self, e):
        self.close_fifo()
        if hasattr(self, 'reader'): self.reader.stop()
        e.accept()

def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion") 
    win = RobotUI()
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()

