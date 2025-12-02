#!/usr/bin/env python3
import sys
import os
import time
import math
import errno
from dataclasses import dataclass
from typing import Optional, List

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
STEPS_MOTOR = 200.0   
MICROSTEPS = 8.0      
REDUCCION_EJE_1 = 12.12  
REDUCCION_EJE_2 = 16.0  
REDUCCION_EJE_3 = 16.0  

PASOS_POR_GRADO_1 = (STEPS_MOTOR * MICROSTEPS * REDUCCION_EJE_1) / 360.0
PASOS_POR_GRADO_2 = (STEPS_MOTOR * MICROSTEPS * REDUCCION_EJE_2) / 360.0
PASOS_POR_GRADO_3 = (STEPS_MOTOR * MICROSTEPS * REDUCCION_EJE_3) / 360.0

L_BASE_OFFSET = 200.0
L_BRAZO = 250.0
L_ANTEBRAZO = 250.0

LIM_EJE1 = (-180, 180)
LIM_EJE2 = (-85, 85)
LIM_EJE3 = (-85, 85)

# PUNTOS PREDEFINIDOS (SIEMPRE EN PASOS)
PRESET_POINTS = [
    [1616,  2133,  2133, "P1 (Directo +30°)"], 
    [-1616, -2133, -2133, "P2 (Directo -30°)"], 
    [0,     0,     0,    "Home (0,0,0)"],
    [4848,  0,     0,    "Base 90° Derecha"],      
    [-4848, 0,     0,    "Base 90° Izquierda"],
    [0,     3200,  0,    "Brazo 45° Arriba"],      
    [0,     6400,  0,    "Brazo 90° Vertical"],
    [0,     0,     3200, "Antebrazo 45°"],
    [2693,  2133,  -2133,"Posición Mix A"],    
    [-2693, 4266,  2133, "Posición Mix B"],
]

def compute_units(widget=None):
    return {} 

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

    def calculate_ik(self, tx, ty, tz): return None
    def update_target(self, x, y, z): return False, None

robot = Kinematics()

# -----------------------------------------------------------------
# Hilo lector
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

class SequenceEditorDialog(QtWidgets.QDialog):
    def __init__(self, parent=None, current_seq=None):
        super().__init__(parent)
        self.setWindowTitle("Configurar Secuencia")
        self.resize(600, 350)
        self.setStyleSheet("font-size: 14px;") 
        
        self.selected_indices = current_seq if current_seq else []

        main_layout = QtWidgets.QHBoxLayout(self)
        
        left_layout = QtWidgets.QVBoxLayout()
        left_layout.addWidget(QtWidgets.QLabel("<b>Disponibles:</b>"))
        self.list_available = QtWidgets.QListWidget()
        for i, pt in enumerate(PRESET_POINTS):
            self.list_available.addItem(f"{i}. {pt[3]}")
        left_layout.addWidget(self.list_available)
        
        center_layout = QtWidgets.QVBoxLayout()
        center_layout.addStretch()
        self.btn_add = QtWidgets.QPushButton(">>")
        self.btn_remove = QtWidgets.QPushButton("<<")
        self.btn_clear = QtWidgets.QPushButton("Limpiar")
        
        style_center = "padding: 8px; font-weight: bold;"
        self.btn_add.setStyleSheet(style_center + "background-color: #2196F3; color: white;")
        self.btn_remove.setStyleSheet(style_center + "background-color: #F44336; color: white;")
        self.btn_clear.setStyleSheet(style_center + "background-color: #9E9E9E; color: black;")
        
        center_layout.addWidget(self.btn_add)
        center_layout.addWidget(self.btn_remove)
        center_layout.addWidget(self.btn_clear)
        
        self.btn_save = QtWidgets.QPushButton("GUARDAR")
        self.btn_save.setStyleSheet("padding: 15px; font-weight: bold; background-color: #4CAF50; color: white; border: 2px solid #333; border-radius: 8px; margin-top: 20px;")
        
        center_layout.addSpacing(20)
        center_layout.addWidget(self.btn_save)
        center_layout.addStretch()
        
        right_layout = QtWidgets.QVBoxLayout()
        right_layout.addWidget(QtWidgets.QLabel("<b>Ejecutar:</b>"))
        self.list_sequence = QtWidgets.QListWidget()
        right_layout.addWidget(self.list_sequence)
        
        btn_up = QtWidgets.QPushButton("▲")
        btn_down = QtWidgets.QPushButton("▼")
        right_layout.addWidget(btn_up)
        right_layout.addWidget(btn_down)

        self._refresh_sequence_list()

        main_layout.addLayout(left_layout)
        main_layout.addLayout(center_layout)
        main_layout.addLayout(right_layout)
        
        self.setLayout(main_layout)

        self.btn_add.clicked.connect(self._add_item)
        self.btn_remove.clicked.connect(self._remove_item)
        self.btn_clear.clicked.connect(self._clear_all)
        btn_up.clicked.connect(self._move_up)
        btn_down.clicked.connect(self._move_down)
        self.btn_save.clicked.connect(self.accept)

    def _add_item(self):
        row = self.list_available.currentRow()
        if row >= 0:
            self.selected_indices.append(row)
            self._refresh_sequence_list()

    def _remove_item(self):
        row = self.list_sequence.currentRow()
        if row >= 0:
            self.selected_indices.pop(row)
            self._refresh_sequence_list()

    def _clear_all(self):
        self.selected_indices = []
        self._refresh_sequence_list()

    def _move_up(self):
        row = self.list_sequence.currentRow()
        if row > 0:
            self.selected_indices[row], self.selected_indices[row-1] = self.selected_indices[row-1], self.selected_indices[row]
            self._refresh_sequence_list()
            self.list_sequence.setCurrentRow(row-1)

    def _move_down(self):
        row = self.list_sequence.currentRow()
        if row < len(self.selected_indices) - 1 and row >= 0:
            self.selected_indices[row], self.selected_indices[row+1] = self.selected_indices[row+1], self.selected_indices[row]
            self._refresh_sequence_list()
            self.list_sequence.setCurrentRow(row+1)

    def _refresh_sequence_list(self):
        self.list_sequence.clear()
        for i, idx in enumerate(self.selected_indices):
            name = PRESET_POINTS[idx][3]
            self.list_sequence.addItem(f"{i+1}. {name}")

    def get_sequence_indices(self):
        return self.selected_indices

# --- MANUAL DIALOG ---
class ManualDialog(QtWidgets.QDialog):
    def __init__(self, parent=None, sender_callback=None, stop_callback=None):
        super().__init__(parent)
        self.setWindowTitle("Modo Manual")
        self.sender = sender_callback
        self.stop_global = stop_callback
        self.resize(500, 300)
        
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(10, 10, 10, 10)
        lay.setSpacing(10)
        
        lay.addWidget(QtWidgets.QLabel("<b>Eje 1 (Base):</b>"))
        lay.addLayout(self._create_axis_row(1))
        
        lay.addWidget(QtWidgets.QLabel("<b>Eje 2 (Brazo):</b>"))
        lay.addLayout(self._create_axis_row(2))
        
        lay.addWidget(QtWidgets.QLabel("<b>Eje 3 (Antebrazo):</b>"))
        lay.addLayout(self._create_axis_row(3))
        
        btns = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Close)
        btns.rejected.connect(self.reject)
        lay.addWidget(btns)
        
    def _create_axis_row(self, axis_idx):
        row = QtWidgets.QHBoxLayout()
        style_base = "font-size: 14px; font-weight: bold; padding: 10px; border-radius: 8px; border: 2px solid #333;"
        
        btn_ccw = QtWidgets.QPushButton("◀️ (-)")
        btn_ccw.setStyleSheet(style_base + "background-color: #FF9800; color: white;")
        
        btn_stop = QtWidgets.QPushButton("■ STOP")
        btn_stop.setStyleSheet(style_base + "background-color: #D32F2F; color: white;")
        
        btn_cw = QtWidgets.QPushButton("▶️ (+)")
        btn_cw.setStyleSheet(style_base + "background-color: #FF9800; color: white;")
        
        row.addWidget(btn_ccw)
        row.addWidget(btn_stop)
        row.addWidget(btn_cw)
        
        cmd_base = 600 + axis_idx 
        btn_ccw.pressed.connect(lambda: self.mv(cmd_base, 2))
        btn_ccw.released.connect(lambda: self.mv(cmd_base, 0))
        btn_cw.pressed.connect(lambda: self.mv(cmd_base, 1))
        btn_cw.released.connect(lambda: self.mv(cmd_base, 0))
        if self.stop_global: btn_stop.clicked.connect(self.stop_global)
        return row

    def mv(self, cmd, d):
        if self.sender: self.sender(cmd, d, 0)

class PresetMoveDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Seleccionar Punto")
        self.resize(400, 150)
        lay = QtWidgets.QVBoxLayout(self)
        lay.addWidget(QtWidgets.QLabel("<b>Selecciona objetivo:</b>"))
        self.combo_points = QtWidgets.QComboBox()
        self.combo_points.setStyleSheet("font-size: 16px; padding: 5px;")
        for pt in PRESET_POINTS:
            self.combo_points.addItem(f"{pt[3]}")
        lay.addWidget(self.combo_points)
        btns = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        btns.accepted.connect(self.accept); btns.rejected.connect(self.reject)
        lay.addWidget(btns)
    def get_selected_point_index(self):
        return self.combo_points.currentIndex()

# ==== VENTANA PRINCIPAL ====
class RobotUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Control Robot - Interfaz Tesis")
        self.setMinimumSize(800, 460)
        self.setMaximumSize(800, 480)
        
        # Variables
        self.fd_tx = None
        self.is_homing = False
        self.is_ptp_active = False        # Bandera para PTP simple
        self.is_running_sequence = False  # Bandera para Secuencia
        self.alarm_active = False
        self.ignore_status_until = 0
        self.custom_sequence_indices = [] 
        self.sequence_queue = []  
        self.is_stopped = False
        
        # === FIX: Bandera para evitar condición de carrera ===
        self.waiting_for_start = False

        self.last_a1, self.last_a2, self.last_a3 = 0.0, 0.0, 0.0
        
        # === UI ===
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QGridLayout(central)
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(6)

        # --- COLUMNA 1: PAD ---
        self.btn_up    = self._make_dir_button("▲ Eje 2+", "Arriba", orange=True)
        self.btn_down  = self._make_dir_button("▼ Eje 2-", "Abajo",  orange=True)
        self.btn_left  = self._make_dir_button("◀️ Eje 1-", "Izquierda", orange=True)
        self.btn_right = self._make_dir_button("▶️ Eje 1+", "Derecha",   orange=True)
        self.btn_stop_play = self._make_dir_button("■", "Stop / Play", orange=False)
        self._apply_stop_style()

        pad = QtWidgets.QGridLayout()
        pad.setSpacing(4)
        pad.addWidget(self.btn_up,        0, 1)
        pad.addWidget(self.btn_left,      1, 0)
        pad.addWidget(self.btn_stop_play, 1, 1)  
        pad.addWidget(self.btn_right,     1, 2)
        pad.addWidget(self.btn_down,      2, 1)
        root.addWidget(self._group("Manual Rápido", pad), 0, 0, 2, 1)

        # --- COLUMNA 2: CENTRO ---
        col_center = QtWidgets.QVBoxLayout()
        col_center.setSpacing(6)

        # Status Box
        self.lbl_status = QtWidgets.QLabel("IDLE")
        self.lbl_status.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_status.setStyleSheet("font-size: 20px; font-weight: bold; color: white; background-color: green; border-radius: 6px; padding: 2px; border: 2px solid #333;")
        self.lbl_status.setFixedHeight(30)
        col_center.addWidget(self.lbl_status)

        # Info Posición
        self.lbl_ang = QtWidgets.QLabel("A: 0.0°, 0.0°, 0.0°")
        self.lbl_ang.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_ang.setStyleSheet("font-size: 16px; font-weight: bold; color: #333; background-color: #E0E0E0; border-radius: 5px; padding: 4px; border: 1px solid #999;")
        col_center.addWidget(self._group("Posición", self.lbl_ang))
        self.lbl_xyz = QtWidgets.QLabel("") 

        # Slider
        slider_layout = QtWidgets.QVBoxLayout()
        slider_layout.setSpacing(2)
        self.lbl_vel_val = QtWidgets.QLabel("50 %")
        self.lbl_vel_val.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_vel_val.setStyleSheet("font-weight: bold; font-size: 14px; color: #333;")
        
        self.slider_speed = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider_speed.setRange(1, 100) 
        self.slider_speed.setValue(50)
        self.slider_speed.setFixedHeight(20)
        
        slider_layout.addWidget(self.slider_speed)
        slider_layout.addWidget(self.lbl_vel_val)
        col_center.addWidget(self._group("Velocidad", slider_layout))

        # Botones Modos
        btn_layout = QtWidgets.QVBoxLayout()
        btn_layout.setSpacing(4)
        btn_row = QtWidgets.QHBoxLayout()
        self.btn_manual = self._make_round_button("JOG", "Manual")
        self.btn_ptp    = self._make_round_button("PTP", "Puntos")
        btn_row.addWidget(self.btn_manual)
        btn_row.addWidget(self.btn_ptp)
        
        self.btn_config_seq = QtWidgets.QPushButton("CREAR SECUENCIA")
        self.btn_config_seq.setMinimumHeight(25) 
        self.btn_config_seq.setStyleSheet("background-color: #FFC107; color: black; font-weight: bold; font-size: 12px; border-radius: 6px; border: 2px solid #333;")
        
        self.btn_seq = QtWidgets.QPushButton("EJECUTAR SECUENCIA")
        self.btn_seq.setMinimumHeight(30) 
        self.btn_seq.setStyleSheet("background-color: #9C27B0; color: white; font-weight: bold; font-size: 14px; border-radius: 6px; border: 2px solid #333;")

        btn_layout.addLayout(btn_row)
        btn_layout.addWidget(self.btn_config_seq)
        btn_layout.addWidget(self.btn_seq)
        col_center.addWidget(self._group("Operación", btn_layout))
        
        # Home
        self.btn_home = QtWidgets.QPushButton("PUESTA A CERO")
        self.btn_home.setMinimumHeight(35) 
        self.btn_home.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; font-size: 16px; border-radius: 6px; border: 2px solid #333;")
        col_center.addWidget(self.btn_home)

        root.addLayout(col_center, 0, 1, 2, 1)

        # --- LOG ---
        self.log = QtWidgets.QPlainTextEdit(readOnly=True)
        self.log.setMaximumHeight(60)
        self.log.setStyleSheet("font-family: Monospace; font-size: 10px;")
        log_box = self._group("Log", QtWidgets.QVBoxLayout()); log_box.layout().addWidget(self.log)
        root.addWidget(log_box, 2, 0, 1, 2)

        # --- INIT ---
        self._wire_buttons()
        self.setup_reader()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.poll_status)
        self.timer.start(1000)
        
        QtCore.QTimer.singleShot(500, self.update_speed)
        self.update_ui()

    # ---------- Helpers UI ----------
    def _group(self, title, inner):
        box = QtWidgets.QGroupBox(title)
        box.setStyleSheet("QGroupBox { font-weight: bold; font-size: 11px; color: #333; border: 1px solid #aaa; border-radius: 3px; margin-top: 6px; } QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 3px; }")
        if isinstance(inner, QtWidgets.QLayout): 
            inner.setContentsMargins(2, 8, 2, 2)
            box.setLayout(inner)
        elif isinstance(inner, QtWidgets.QWidget): 
             lay = QtWidgets.QVBoxLayout()
             lay.setContentsMargins(2, 8, 2, 2)
             lay.addWidget(inner)
             box.setLayout(lay)
        return box

    def _make_dir_button(self, text, tip, orange=False):
        btn = QtWidgets.QPushButton(text)
        btn.setToolTip(tip)
        btn.setFixedSize(65, 65) 
        font = btn.font(); font.setPointSize(10); font.setBold(True); btn.setFont(font)
        color = "#FF9800" if orange else "#F44336" 
        hover = "#F57C00" if orange else "#D32F2F"
        btn.setStyleSheet(f"QPushButton {{ background-color: {color}; color: white; border-radius: 8px; border: 2px solid #333; }} QPushButton:hover {{ background-color: {hover}; }} QPushButton:pressed {{ background-color: #333; }}")
        return btn

    def _make_round_button(self, text, tip):
        btn = QtWidgets.QPushButton(text)
        btn.setToolTip(tip)
        btn.setFixedSize(55, 55) 
        btn.setStyleSheet("QPushButton { background-color: #607D8B; color: white; border-radius: 27px; font-weight: bold; font-size: 11px; border: 2px solid #333; } QPushButton:hover { background-color: #78909C; } QPushButton:pressed { background-color: #37474F; }")
        return btn

    def _apply_stop_style(self):
        if self.is_stopped:
            self.btn_stop_play.setText("▶️")
            self.btn_stop_play.setStyleSheet("background-color: #4CAF50; color: white; border-radius: 8px; font-size: 20px; border: 2px solid #333;")
            self.log_cmd("SISTEMA PAUSADO")
        else:
            self.btn_stop_play.setText("■")
            self.btn_stop_play.setStyleSheet("background-color: #F44336; color: white; border-radius: 8px; font-size: 20px; border: 2px solid #333;")
            self.log_cmd("SISTEMA ACTIVO")

    # ---------- Control ----------
    def _wire_buttons(self):
        self.btn_up.clicked.connect(lambda: self.move_joint(2, 5))
        self.btn_down.clicked.connect(lambda: self.move_joint(2, -5))
        self.btn_left.clicked.connect(lambda: self.move_joint(1, -5))
        self.btn_right.clicked.connect(lambda: self.move_joint(1, 5))
        
        self.btn_home.clicked.connect(self.do_home)
        self.btn_manual.clicked.connect(self._open_manual_dialog)
        self.btn_ptp.clicked.connect(self._open_preset_dialog)
        
        self.btn_config_seq.clicked.connect(self.open_sequence_editor)
        self.btn_seq.clicked.connect(self.start_sequence)
        
        self.btn_stop_play.clicked.connect(self._toggle_stop)
        
        self.slider_speed.valueChanged.connect(lambda v: self.lbl_vel_val.setText(f"{v} %"))
        self.slider_speed.sliderReleased.connect(self.update_speed)

    def _toggle_stop(self):
        self.is_stopped = not self.is_stopped
        self._apply_stop_style()

    def update_speed(self):
        pct = self.slider_speed.value()
        min_hz = 200
        max_hz = 5000
        hz = min_hz + (max_hz - min_hz) * (pct / 100.0)
        self.log_cmd(f"Set Velocidad: {int(hz)} Hz ({pct}%)")
        self.send_spi(700, int(hz), 0)

    # --- Lógica de Secuencia ---
    def open_sequence_editor(self):
        dlg = SequenceEditorDialog(self, self.custom_sequence_indices)
        if run_dialog(dlg) == QtWidgets.QDialog.Accepted:
            self.custom_sequence_indices = dlg.get_sequence_indices()
            self.log_cmd(f"Secuencia guardada: {len(self.custom_sequence_indices)} pasos.")

    def start_sequence(self):
        if self.is_running_sequence or self.is_homing: return
        if not self.custom_sequence_indices:
            QtWidgets.QMessageBox.warning(self, "Error", "La secuencia está vacía.\nUse 'CREAR SECUENCIA' primero.")
            return

        self.log_cmd("=== INICIANDO SECUENCIA ===")
        self.sequence_queue = list(self.custom_sequence_indices)
        self.is_running_sequence = True
        self.process_next_step()

    def process_next_step(self):
        if not self.sequence_queue:
            self.log_cmd("=== SECUENCIA FINALIZADA ===")
            self.is_running_sequence = False
            self.lbl_status.setText("IDLE")
            self.lbl_status.setStyleSheet("background-color: green; color: white; font-weight: bold; padding: 4px; border: 2px solid #333; font-size: 20px;")
            # MENSAJE FINAL SECUENCIA
            self._show_success("Secuencia realizada correctamente")
            return

        idx = self.sequence_queue.pop(0)
        self.execute_preset(idx)

    def _open_preset_dialog(self):
        if self.is_homing: return
        dlg = PresetMoveDialog(self)
        if run_dialog(dlg) == QtWidgets.QDialog.Accepted:
            idx = dlg.get_selected_point_index()
            self.is_ptp_active = True # Flag PTP
            self.execute_preset(idx)

    def execute_preset(self, idx):
        steps1, steps2, steps3, name = PRESET_POINTS[idx]
        self.log_cmd(f"PTP: {name}")
        
        t1_deg = steps1 / PASOS_POR_GRADO_1
        t2_deg = steps2 / PASOS_POR_GRADO_2
        t3_deg = steps3 / PASOS_POR_GRADO_3
        
        robot.theta1, robot.theta2, robot.theta3 = t1_deg, t2_deg, t3_deg
        self.update_ui()
        self.send_kinematic_move_steps(steps1, steps2, steps3)

    def _open_manual_dialog(self):
        run_dialog(ManualDialog(self, self.send_spi, self._toggle_stop))

    def move_joint(self, axis, delta):
        if self.is_homing: return
        a1, a2, a3 = robot.theta1, robot.theta2, robot.theta3
        if axis == 1: a1 += delta
        elif axis == 2: a2 += delta
        elif axis == 3: a3 += delta
        
        robot.theta1, robot.theta2, robot.theta3 = a1, a2, a3
        self.update_ui()
        
        s1 = int((a1 - self.last_a1) * PASOS_POR_GRADO_1)
        s2 = int((a2 - self.last_a2) * PASOS_POR_GRADO_2)
        s3 = int((a3 - self.last_a3) * PASOS_POR_GRADO_3)
        self.last_a1, self.last_a2, self.last_a3 = a1, a2, a3
        
        if s1==0 and s2==0 and s3==0: return
        self.send_spi(400, s1, s2); time.sleep(0.002); self.send_spi(401, s3, 0)

    def send_kinematic_move_steps(self, dest_s1, dest_s2, dest_s3):
        curr_s1 = self.last_a1 * PASOS_POR_GRADO_1
        curr_s2 = self.last_a2 * PASOS_POR_GRADO_2
        curr_s3 = self.last_a3 * PASOS_POR_GRADO_3
        
        d1 = int(dest_s1 - curr_s1)
        d2 = int(dest_s2 - curr_s2)
        d3 = int(dest_s3 - curr_s3)
        
        self.last_a1 = dest_s1 / PASOS_POR_GRADO_1
        self.last_a2 = dest_s2 / PASOS_POR_GRADO_2
        self.last_a3 = dest_s3 / PASOS_POR_GRADO_3
        
        if d1==0 and d2==0 and d3==0:
             if self.is_running_sequence: QtCore.QTimer.singleShot(100, self.process_next_step)
             elif self.is_ptp_active: # Si terminó y no era secuencia
                 self.is_ptp_active = False
                 self._show_success("Punto realizado correctamente")
             return
        
        # === FIX: Activar espera de confirmación de movimiento ===
        self.waiting_for_start = True 
             
        self.send_spi(400, d1, d2)
        time.sleep(0.002)
        self.send_spi(401, d3, 0)
        # Status Moving (Naranja)
        self.lbl_status.setText("MOVING...")
        self.lbl_status.setStyleSheet("background-color: orange; color: white; font-weight: bold; padding: 4px; border: 2px solid #333; font-size: 20px;")

    def do_home(self):
        self.send_spi(500, 0, 0)
        self.last_a1, self.last_a2, self.last_a3 = 0.0, 0.0, 0.0
        robot.theta1, robot.theta2, robot.theta3 = 0.0, 0.0, 0.0
        self.update_ui()

    def update_ui(self):
        self.lbl_ang.setText(f"A: {robot.theta1:.1f}°, {robot.theta2:.1f}°, {robot.theta3:.1f}°")

    def connect_fifo(self):
        if self.fd_tx is not None: return
        if not os.path.exists(FIFO_PATH): return 
        try:
            self.fd_tx = os.open(FIFO_PATH, os.O_WRONLY | os.O_NONBLOCK)
            self.log_cmd("FIFO TX conectado.")
        except OSError: pass

    def close_fifo(self):
        if self.fd_tx:
            try: os.close(self.fd_tx)
            except: pass
            self.fd_tx = None

    def send_spi(self, cmd, a1, a2):
        if not self.fd_tx: self.connect_fifo()
        if not self.fd_tx: return
        try:
            os.write(self.fd_tx, f"I {int(cmd)} {int(a1)} {int(a2)}\n".encode())
            if cmd != 0: self.log_cmd(f"TX: {cmd} {a1} {a2}")
        except OSError: self.close_fifo()

    def log_cmd(self, t):
        if hasattr(self, 'log'):
            self.log.appendPlainText(f">> {t}")
            c = self.log.textCursor(); c.movePosition(QtGui.QTextCursor.End); self.log.setTextCursor(c)

    def poll_status(self):
        if not self.alarm_active: self.send_spi(0, 0, 0)

    def setup_reader(self):
        self.th = QtCore.QThread()
        self.reader = StatusReader()
        self.reader.moveToThread(self.th)
        self.th.started.connect(self.reader.run)
        self.reader.statusReceived.connect(self.on_status)
        self.th.start()

    def _show_success(self, text):
        msg = QtWidgets.QMessageBox(self)
        msg.setWindowTitle("Confirmación")
        msg.setText(text)
        msg.setIconPixmap(self.style().standardIcon(QtWidgets.QStyle.SP_DialogApplyButton).pixmap(64,64))
        msg.exec()

    def on_status(self, s):
        if self.alarm_active or not s: return
        
        if "900" in s:
            self.alarm_active = True
            self.is_running_sequence = False
            self.sequence_queue = []
            self.is_ptp_active = False
            
            # Resetear flag
            self.waiting_for_start = False

            self.lbl_status.setText("¡FALLA!")
            self.lbl_status.setStyleSheet("background-color: red; color: white; font-weight: bold; padding: 4px; border: 2px solid #333; font-size: 20px;")
            
            msg = QtWidgets.QMessageBox(self)
            msg.setIcon(QtWidgets.QMessageBox.Critical); msg.setWindowTitle("FALLA")
            msg.setText("ERROR ENCODERS"); msg.setStandardButtons(QtWidgets.QMessageBox.Ok)
            if msg.exec() == QtWidgets.QMessageBox.Ok:
                self.alarm_active = False
                self.lbl_status.setText("IDLE"); self.lbl_status.setStyleSheet("background-color: green; color: white; font-weight: bold; padding: 4px; border: 2px solid #333; font-size: 20px;")
            return

        # === FIX: Lógica de protección contra condición de carrera ===
        if "500" in s:
            # El robot confirmó que se mueve. Ya podemos dejar de esperar el inicio.
            self.waiting_for_start = False

            # Diferenciar si es HOMING o MOVIMIENTO normal
            if self.is_homing:
                self.lbl_status.setText("HOMING...")
                self.lbl_status.setStyleSheet("background-color: #2196F3; color: white; font-weight: bold; padding: 4px; border: 2px solid #333; font-size: 20px;") 
            else:
                self.lbl_status.setText("MOVING...")
                self.lbl_status.setStyleSheet("background-color: orange; color: white; font-weight: bold; padding: 4px; border: 2px solid #333; font-size: 20px;") 

        elif "501" in s:
            # Si estamos esperando que arranque, ignoramos este mensaje 501
            if self.waiting_for_start:
                return

            # 1. Fin de Homing
            if self.is_homing:
                self.is_homing = False
                self.log_cmd("Homing OK")
                self._show_success("Puesta a cero realizada correctamente")

            # 2. Fin de PTP (Simple)
            elif self.is_ptp_active:
                self.is_ptp_active = False
                self._show_success("Punto realizado correctamente")
            
            # 3. Fin de Paso de Secuencia
            elif self.is_running_sequence:
                QtCore.QTimer.singleShot(100, self.process_next_step)

            # Restore IDLE visual
            if self.lbl_status.text() != "IDLE":
                self.lbl_status.setText("IDLE"); self.lbl_status.setStyleSheet("background-color: green; color: white; font-weight: bold; padding: 4px; border: 2px solid #333; font-size: 20px;")

    def closeEvent(self, e):
        self.close_fifo(); 
        if hasattr(self, 'reader'): self.reader.stop()
        e.accept()

def main():
    app = QtWidgets.QApplication(sys.argv); app.setStyle("Fusion") 
    win = RobotUI(); win.show(); sys.exit(app.exec_())

if __name__ == "__main__":
    main()
