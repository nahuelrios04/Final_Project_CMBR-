import sys
import os
import time
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

# HiDPI tweaks
try:
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
except Exception:
    pass

FIFO_PATH = "/tmp/spi_tx"
FIFO_RX_PATH = "/tmp/spi_rx"

def run_dialog(dlg: QtWidgets.QDialog) -> int:
    return dlg.exec() if hasattr(dlg, "exec") else dlg.exec_()

@dataclass
class Command:
    type: str
    payload: dict

# ==== FIFO helpers ====
def spi_send_three(a: int, b: int, c: int):
    try:
        with open(FIFO_PATH, "a") as f:
            f.write(f"I {int(a)} {int(b)} {int(c)}\n")
    except Exception as e:
        # Ignoramos errores si es solo polling (comando 0) para no saturar la consola
        if a != 0:
            print(f"[SPI-FIFO] No pude escribir: {e}")

def jog_start(direction: str):
    dir_map = {"left":3, "right":4, "up":1, "down":2}
    spi_send_three(210, dir_map.get(direction, 0), 0)

def jog_stop():
    spi_send_three(211, 0, 0)

# ==== sizing ====
def screen_metrics(widget=None):
    app = QtWidgets.QApplication.instance()
    scr = None
    try:
        if widget is not None and hasattr(app, "screenAt"):
            scr = app.screenAt(widget.mapToGlobal(QtCore.QPoint(0, 0)))
    except Exception:
        scr = None
    if scr is None:
        scr = app.primaryScreen()
    g = scr.availableGeometry()
    try:
        dpi = scr.logicalDotsPerInch()
    except Exception:
        dpi = 96.0
    return g.width(), g.height(), dpi

def compute_units(widget=None):
    w,h,dpi = screen_metrics(widget)
    side_min = min(w, h)
    U = max(30, min(int(side_min/13), 100))
    dpi_sf = dpi/96.0 if dpi and dpi>0 else 1.0
    if dpi_sf>0: U = int(U/dpi_sf)
    sizes = {
        "U":U, "btn_big":int(U*1.45), "btn_small":int(U*0.95),
        "pad":int(U*0.26), "radius":int(U*0.22),
        "font_delta_big":2, "font_delta_norm":1,
    }
    sizes["btn_big"]   = max(54, min(sizes["btn_big"], 120))
    sizes["btn_small"] = max(38, min(sizes["btn_small"], 86))
    sizes["pad"]       = max(6,  min(sizes["pad"], 16))
    sizes["radius"]    = max(6,  min(sizes["radius"], 12))
    return sizes

# ==== Diálogos ====
class ManualDialog(QtWidgets.QDialog):
    def __init__(self, parent=None, base_speed=50):
        super().__init__(parent)
        self.setWindowTitle("Modo manual")
        self.setModal(True)
        S = compute_units(self)
        
        # Este es el valor que devolveremos
        self._value = 0
        self.base_speed = base_speed # <-- AÑADIDO

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(S["pad"], S["pad"], S["pad"], S["pad"])
        lay.setSpacing(int(S["pad"]*0.7))

        self.info = QtWidgets.QLabel("Seleccioná el eje y activá el movimiento.") # Texto cambiado
        self.info.setWordWrap(True)

        axis_row = QtWidgets.QHBoxLayout()
        axis_row.addWidget(QtWidgets.QLabel("Eje a mover:"))
        self.axis_combo = QtWidgets.QComboBox()
        self.axis_combo.addItems([f"Eje {i}" for i in range(1,4)]) # Ejes 1, 2, 3
        axis_row.addWidget(self.axis_combo)
        axis_box = QtWidgets.QGroupBox("Selección de eje")
        axis_box.setLayout(axis_row)

        # --- REEMPLAZO POR BOTONES ESTILO FLECHA ---
        btn_layout = QtWidgets.QHBoxLayout()
        
        # Copiamos la lógica de estilo de la ventana principal (parent)
        # Asumimos que parent es RobotUI
        parent_S = self.parent().S if self.parent() and hasattr(self.parent(), "S") else S
        S_btn_size = parent_S["btn_big"]
        S_pad = max(6, int(parent_S["pad"]*0.6))
        S_radius = parent_S["radius"]
        S_font = self.parent()._button_font(self.font(), True) if self.parent() and hasattr(self.parent(), "_button_font") else self.font()

        # Botón Negativo (◀)
        self.btn_neg = QtWidgets.QPushButton("◀")
        self.btn_neg.setToolTip("Mover eje en dirección negativa")
        self.btn_neg.setMinimumSize(S_btn_size, S_btn_size)
        self.btn_neg.setFont(S_font)
        self.btn_neg.setStyleSheet(
            "QPushButton { background-color:#ff9800; color:#fff;"
            f"  padding:{S_pad}px {S_pad+4}px; border:none; border-radius:{S_radius}px; font-weight:700; }}"
            "QPushButton:hover{ background-color:#fb8c00; }"
            "QPushButton:pressed{ background-color:#e65100; }"
        )

        # Botón Stop (■)
        self.btn_stop = QtWidgets.QPushButton("■")
        self.btn_stop.setToolTip("Detener eje")
        self.btn_stop.setMinimumSize(S_btn_size, S_btn_size)
        self.btn_stop.setFont(S_font)
        self.btn_stop.setStyleSheet(
            "QPushButton { background-color:#c62828; color:#fff; border:none;"
            f"  padding:{S_pad}px {S_pad+4}px; border-radius:{S_radius}px; font-weight:700; }}"
            "QPushButton:hover{ background-color:#d32f2f; }"
            "QPushButton:pressed{ background-color:#b71c1c; }"
        )

        # Botón Positivo (▶)
        self.btn_pos = QtWidgets.QPushButton("▶")
        self.btn_pos.setToolTip("Mover eje en dirección positiva")
        self.btn_pos.setMinimumSize(S_btn_size, S_btn_size)
        self.btn_pos.setFont(S_font)
        self.btn_pos.setStyleSheet(
            "QPushButton { background-color:#ff9800; color:#fff;"
            f"  padding:{S_pad}px {S_pad+4}px; border:none; border-radius:{S_radius}px; font-weight:700; }}"
            "QPushButton:hover{ background-color:#fb8c00; }"
            "QPushButton:pressed{ background-color:#e65100; }"
        )

        btn_layout.addWidget(self.btn_neg)
        btn_layout.addWidget(self.btn_stop)
        btn_layout.addWidget(self.btn_pos)
        # --- FIN REEMPLAZO ---

        # Nuevo label de estado
        self.status_label = QtWidgets.QLabel("VELOCIDAD: 0")
        self.status_label.setAlignment(QtCore.Qt.AlignCenter)
        f = self.status_label.font(); f.setPointSize(f.pointSize()+S["font_delta_norm"]); f.setBold(True)
        self.status_label.setFont(f)

        btns = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok|QtWidgets.QDialogButtonBox.Cancel)

        lay.addWidget(self.info)
        lay.addWidget(axis_box)
        
        # Añadir los nuevos widgets
        lay.addLayout(btn_layout)
        lay.addWidget(self.status_label)
        
        lay.addWidget(btns)

        # Conectar botones
        self.btn_neg.clicked.connect(lambda: self._set_speed(-self.base_speed)) # <-- MODIFICADO
        self.btn_pos.clicked.connect(lambda: self._set_speed(self.base_speed))  # <-- MODIFICADO
        self.btn_stop.clicked.connect(lambda: self._set_speed(0))   # 0 para stop

        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        
        self._set_speed(0) # Estado inicial

    def _set_speed(self, speed):
        self._value = speed
        self.status_label.setText(f"VELOCIDAD: {speed}")
        if speed == 0:
            self.status_label.setStyleSheet("color: #c62828; font-weight: 700;") # Rojo
        else:
            self.status_label.setStyleSheet("color: #2e7d32; font-weight: 700;") # Verde

    def _activate(self):
        # Esta función ya no se usa, pero la dejamos por si acaso
        self._set_speed(50)
    def _deactivate(self):
        # Esta función ya no se usa
        self._set_speed(0)

    def value(self) -> int: 
        return int(self._value) # Devuelve el valor guardado
    
    def axis(self) -> int:  
        return self.axis_combo.currentIndex()+1

class SpeedDialog(QtWidgets.QDialog):
    def __init__(self, parent=None, initial_value=50):
        super().__init__(parent)
        self.setWindowTitle("Velocidad")
        self.setModal(True)
        S = compute_units(self)

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(S["pad"], S["pad"], S["pad"], S["pad"])
        lay.setSpacing(int(S["pad"]*0.7))

        self.lbl = QtWidgets.QLabel("Seleccioná la velocidad (0–100).")
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal); self.slider.setRange(0,100)
        self.slider.setSingleStep(1); self.slider.setPageStep(5)

        self.spin = QtWidgets.QSpinBox(); self.spin.setRange(0,100)
        self.slider.valueChanged.connect(self.spin.setValue)
        self.spin.valueChanged.connect(self.slider.setValue)
        self.slider.setValue(initial_value) # <-- MODIFICADO

        btns = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok|QtWidgets.QDialogButtonBox.Cancel)
        lay.addWidget(self.lbl); lay.addWidget(self.slider); lay.addWidget(self.spin); lay.addWidget(btns)
        btns.accepted.connect(self.accept); btns.rejected.connect(self.reject)

    def value(self) -> int: return int(self.spin.value())

# -----------------------------------------------------------------
# Hilo para LEER el estado desde el C Bridge
# -----------------------------------------------------------------
class StatusReader(QtCore.QObject):
    # Señales correctas para qtpy/PyQt5/PySide6
    statusReceived = QtCore.pyqtSignal(str)  # <-- CAMBIADO A pyqtSignal
    finished = QtCore.pyqtSignal()           # <-- CAMBIADO A pyqtSignal

    def __init__(self):
        super().__init__()
        self.is_running = True
        print(f"[Reader] Hilo lector inicializado.")

    def run(self):
        print(f"[Reader] Iniciando bucle de lectura en {FIFO_RX_PATH}...")
        while self.is_running:
            try:
                # Si no existe el FIFO, esperar un poco para no saturar CPU
                if not os.path.exists(FIFO_RX_PATH):
                        time.sleep(1)
                        continue

                # Abrir en modo bloqueo (esperará a que el C bridge lo abra)
                with open(FIFO_RX_PATH, 'r') as fifo_rx:
                    print(f"[Reader] FIFO conectado. Escuchando...")
                    while self.is_running:
                        line = fifo_rx.readline()
                        if line:
                            status = line.strip()
                            if status:
                                self.statusReceived.emit(status)
                        else:
                            # EOF: el otro lado cerró el FIFO
                            if self.is_running:
                                print("[Reader] FIFO cerrado por el otro extremo. Reintentando...")
                                time.sleep(1)
                            break 
            except Exception as e:
                if self.is_running:
                    print(f"[Reader] Error de lectura: {e}")
                    time.sleep(2)
        
        print("[Reader] Bucle terminado.")
        self.finished.emit()

    def stop(self):
        self.is_running = False

# ==== Ventana principal ====
class RobotUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Arm Controller")
        self.S = compute_units(self)
        self.is_stopped = False
        self.current_speed = 50 # <-- AÑADIDO: Guardar estado de velocidad

        self.btn_up      = self._make_dir_button("▲", "Arriba", True)
        self.btn_down    = self._make_dir_button("▼", "Abajo",  True)
        self.btn_left    = self._make_dir_button("◀", "Izquierda", True)
        self.btn_right   = self._make_dir_button("▶", "Derecha",    True)

        self.btn_stop_play = self._make_dir_button("■", "Stop / Play (Espacio)", False)
        self._apply_stop_style()
        self.btn_pause = self._make_dir_button("PAUSE", "Pausa", False)

        self.btn_manual = self._make_round_button("MM", "MODO MANUAL")
        self.btn_speed  = self._make_round_button("VEL", "VELOCIDAD")

        self.x_in = self._make_spin("X (mm)", -10000, 10000, 0.1, 0.0)
        self.y_in = self._make_spin("Y (mm)", -10000, 10000, 0.1, 0.0)
        self.z_in = self._make_spin("Z (mm)", -10000, 10000, 0.1, 0.0)

        self.btn_move_abs = QtWidgets.QPushButton("Move To (X, Y, Z)")
        self.btn_home     = QtWidgets.QPushButton("Home")
        self.log          = QtWidgets.QPlainTextEdit(readOnly=True)
        self.log.setPlaceholderText("Log de comandos…")

        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        root = QtWidgets.QGridLayout(central)
        root.setContentsMargins(self.S["pad"], self.S["pad"], self.S["pad"], self.S["pad"])
        root.setHorizontalSpacing(int(self.S["pad"]*0.8)); root.setVerticalSpacing(int(self.S["pad"]*0.8))

        pad = QtWidgets.QGridLayout()
        pad.setHorizontalSpacing(int(self.S["pad"]*0.6)); pad.setVerticalSpacing(int(self.S["pad"]*0.6))
        pad.addWidget(self.btn_up,        0, 1)
        pad.addWidget(self.btn_left,      1, 0)
        pad.addWidget(self.btn_stop_play, 1, 1)
        pad.addWidget(self.btn_right,     1, 2)
        pad.addWidget(self.btn_down,      2, 1)
        pad.addWidget(self.btn_pause,     3, 1)
        root.addWidget(self._group("Direcciones", pad), 0, 0)

        col_circ = QtWidgets.QVBoxLayout()
        col_circ.setSpacing(int(self.S["pad"]*0.6))
        col_circ.addWidget(self.btn_manual, alignment=QtCore.Qt.AlignTop|QtCore.Qt.AlignHCenter)
        col_circ.addWidget(self.btn_speed,  alignment=QtCore.Qt.AlignTop|QtCore.Qt.AlignHCenter)
        col_circ.addStretch(1)
        circ_box = self._group("Controles", col_circ)
        circ_box.setMaximumWidth(int(self.S["btn_big"]*1.5))
        root.addWidget(circ_box, 0, 1)

        form = QtWidgets.QFormLayout()
        form.addRow(self.x_in["label"], self.x_in["spin"])
        form.addRow(self.y_in["label"], self.y_in["spin"])
        form.addRow(self.z_in["label"], self.z_in["spin"])
        root.addWidget(self._group("Entradas", form), 0, 2)

        actions = QtWidgets.QHBoxLayout()
        actions.addWidget(self.btn_move_abs); actions.addWidget(self.btn_home)
        root.addWidget(self._group("Acciones", actions), 1, 0, 1, 3)

        log_box = self._group("Consola", QtWidgets.QVBoxLayout())
        log_box.layout().addWidget(self.log)
        root.addWidget(log_box, 2, 0, 1, 3)

        self._wire_buttons()
        self._install_shortcuts()

        # --- Inicialización de estado y lector ---
        self.is_homing = False
        self.setup_status_reader_thread()
        
        # Timer para polling periódico
        self.poll_timer = QtCore.QTimer(self)
        self.poll_timer.timeout.connect(self.poll_status)
        self.poll_timer.start(250)

        for b in (self.btn_up, self.btn_down, self.btn_left, self.btn_right,
                  self.btn_stop_play, self.btn_pause, self.btn_move_abs, self.btn_home):
            b.setMinimumHeight(int(self.S["btn_big"]*0.55))
        for spin in (self.x_in, self.y_in, self.z_in):
            spin["spin"].setMinimumWidth(int(self.S["btn_small"]*1.8))

        try: self.showMaximized()
        except Exception: self.resize(980,560)

    def _group(self, title, inner_layout):
        box = QtWidgets.QGroupBox(title); box.setLayout(inner_layout); return box
    def _button_font(self, base_font, big=False):
        f = QtGui.QFont(base_font); f.setBold(True)
        f.setPointSize(f.pointSize() + (self.S["font_delta_big"] if big else self.S["font_delta_norm"]))
        return f
    def _make_dir_button(self, text, tooltip, orange=False):
        btn = QtWidgets.QPushButton(text); btn.setToolTip(tooltip)
        size = self.S["btn_big"]; btn.setMinimumSize(size, size); btn.setFont(self._button_font(btn.font(), True))
        pad = max(6, int(self.S["pad"]*0.6)); radius = self.S["radius"]
        base = ( "QPushButton {"
                 f"  padding:{pad}px {pad+4}px; border:none; border-radius:{radius}px; }}"
                 "QPushButton:pressed { background-color:#ccc; } " ) # <-- FIX DE CSS
        if orange:
            base += "QPushButton { background-color:#ff9800; color:#fff; }" \
                    "QPushButton:hover{ background-color:#fb8c00; }" \
                    "QPushButton:pressed{ background-color:#e65100; }"
        else:
            base += "QPushButton { background-color:#e0e0e0; color:#222; }" \
                    "QPushButton:hover{ background-color:#d5d5d5; }" \
                    "QPushButton:pressed{ background-color:#bdbdbd; }"
        btn.setStyleSheet(base); return btn
    def _apply_stop_style(self):
        pad = max(6, int(self.S["pad"]*0.6)); radius = self.S["radius"]
        if self.is_stopped:
            self.btn_stop_play.setText("▶"); self.btn_stop_play.setToolTip("Play / Reanudar (Espacio)")
            self.btn_stop_play.setStyleSheet(
                "QPushButton { background-color:#2e7d32; color:#fff; border:none;"
                f"  padding:{pad}px {pad+4}px; border-radius:{radius}px; font-weight:700; }}"
                "QPushButton:hover{ background-color:#388e3c; }"
                "QPushButton:pressed{ background-color:#1b5e20; }")
        else:
            self.btn_stop_play.setText("■"); self.btn_stop_play.setToolTip("Stop (Espacio)")
            self.btn_stop_play.setStyleSheet(
                "QPushButton { background-color:#c62828; color:#fff; border:none;"
                f"  padding:{pad}px {pad+4}px; border-radius:{radius}px; font-weight:700; }}"
                "QPushButton:hover{ background-color:#d32f2f; }"
                "QPushButton:pressed{ background-color:#b71c1c; }")
    def _make_round_button(self, text, tooltip):
        btn = QtWidgets.QPushButton(text); btn.setToolTip(tooltip)
        size = self.S["btn_small"]; btn.setFixedSize(size, size)
        btn.setFont(self._button_font(btn.font(), False))
        border = max(2, int(size*0.03))
        btn.setStyleSheet(
            "QPushButton { background-color:#e0e0e0; color:#333;"
            f"  border-radius:{size//2}px; border:{border}px solid #9e9e9e; font-weight:700; }}"
            "QPushButton:hover{ background-color:#d5d5d5; }"
            "QPushButton:pressed{ background-color:#cfcfcf; }")
        return btn
    def _make_spin(self, label, mn, mx, step, value):
        spin = QtWidgets.QDoubleSpinBox(); spin.setRange(mn, mx)
        spin.setDecimals(3 if step<1 else 1); spin.setSingleStep(step); spin.setValue(value)
        spin.setAlignment(QtCore.Qt.AlignRight)
        f = spin.font(); f.setPointSize(f.pointSize()+self.S["font_delta_norm"]); spin.setFont(f)
        lab = QtWidgets.QLabel(label); f2 = lab.font(); f2.setPointSize(f2.pointSize()+self.S["font_delta_norm"]); lab.setFont(f2)
        return {"label": lab, "spin": spin}

    def _install_shortcuts(self):
        QtWidgets.QShortcut(QtGui.QKeySequence("Left"),  self, activated=lambda: jog_start("left"))
        QtWidgets.QShortcut(QtGui.QKeySequence("Right"), self, activated=lambda: jog_start("right"))
        QtWidgets.QShortcut(QtGui.QKeySequence("Space"), self, activated=self._toggle_stop_play)
        # --- SIMULACIÓN: Tecla 'H' para forzar fin de Homing ---
        QtWidgets.QShortcut(QtGui.QKeySequence("H"), self, activated=self._simular_fin_homing)

    def _simular_fin_homing(self):
        print("[SIM] Tecla 'H' presionada: Forzando fin de Homing.")
        self.is_homing = True # Forzamos estado para que el handler lo acepte
        self.handle_status("501")

    def _wire_buttons(self):
        self.btn_left.pressed.connect(lambda: (jog_start("left"),  self.send_command(Command("nudge", {"dir": "left"}))))
        self.btn_left.released.connect(jog_stop)
        self.btn_right.pressed.connect(lambda: (jog_start("right"), self.send_command(Command("nudge", {"dir": "right"}))))
        self.btn_right.released.connect(jog_stop)
        self.btn_up.pressed.connect(lambda: (jog_start("up"),    self.send_command(Command("nudge", {"dir": "up"}))))
        self.btn_up.released.connect(jog_stop)
        self.btn_down.pressed.connect(lambda: (jog_start("down"), self.send_command(Command("nudge", {"dir": "down"}))))
        self.btn_down.released.connect(jog_stop)
        self.btn_stop_play.clicked.connect(self._toggle_stop_play)
        self.btn_pause.clicked.connect(self._pause)
        self.btn_manual.clicked.connect(self._open_manual_dialog)
        self.btn_speed.clicked.connect(self._open_speed_dialog)
        self.btn_move_abs.clicked.connect(self._move_abs)
        self.btn_home.clicked.connect(self._home)

    def _toggle_stop_play(self):
        if not self.is_stopped:
            self.send_command(Command("stop", {})); self.is_stopped = True
            spi_send_three(101, 0, 0)
        else:
            self.send_command(Command("play", {})); self.is_stopped = False
            spi_send_three(100, 1, 0)
        self._apply_stop_style()

    def _pause(self):
        self.send_command(Command("pause", {})); spi_send_three(102, 0, 0)

    def _move_abs(self):
        x = float(self.x_in["spin"].value()); y = float(self.y_in["spin"].value()); z = float(self.z_in["spin"].value())
        self.send_command(Command("move_abs", {"x":x, "y":y, "z":z}))
        spi_send_three(400, int(x*1000), int(y*1000))

    def _home(self):
        if self.is_homing:
            self.log.appendPlainText("WARN: Puesta a 0 ya está en progreso.")
            return
        self.send_command(Command("home", {}))
        spi_send_three(500, 0, 0)
        self.is_homing = True
        self.btn_home.setText("Puesta a 0...")
        self.btn_home.setEnabled(False)

    def _open_manual_dialog(self):
        dlg = ManualDialog(self, base_speed=self.current_speed) # <-- MODIFICADO
        if run_dialog(dlg) == QtWidgets.QDialog.Accepted:
            val = dlg.value(); axis = dlg.axis(); enabled = 1 if val!=0 else 0 # <-- MODIFICADO
            self.send_command(Command("manual_enable", {"enabled":enabled, "level":val, "axis":axis}))
            spi_send_three(600, axis, val)

    def _open_speed_dialog(self):
        dlg = SpeedDialog(self, initial_value=self.current_speed) # <-- MODIFICADO
        if run_dialog(dlg) == QtWidgets.QDialog.Accepted:
            val = dlg.value()
            self.current_speed = val # <-- AÑADIDO
            self.send_command(Command("set_speed_pct", {"value":val}))
            spi_send_three(300, val, 0)

    def send_command(self, cmd: Command):
        t = cmd.type; p = cmd.payload
        if t=="nudge": line=f"NUDGE,{p['dir']}"
        elif t=="move_abs": line=f"MOVE,{p['x']:.3f},{p['y']:.3f},{p['z']:.3f}"
        elif t=="home": line="HOME"
        elif t=="stop": line="STOP"
        elif t=="play": line="PLAY"
        elif t=="pause": line="PAUSE"
        elif t=="manual_enable": line=f"MANUAL,{p['enabled']},{p['level']},{p.get('axis',1)}"
        elif t=="set_speed_pct": line=f"SPEED_PCT,{p['value']}"
        else: line=f"UNKNOWN,{t}"
        self.log.appendPlainText(line)
        c = self.log.textCursor(); c.movePosition(QtGui.QTextCursor.End); self.log.setTextCursor(c)

    def setup_status_reader_thread(self):
        self.reader_thread = QtCore.QThread()
        self.status_reader = StatusReader()
        self.status_reader.moveToThread(self.reader_thread)
        self.reader_thread.started.connect(self.status_reader.run)
        self.status_reader.statusReceived.connect(self.handle_status)
        self.status_reader.finished.connect(self.reader_thread.quit)
        self.status_reader.finished.connect(self.status_reader.deleteLater)
        self.reader_thread.finished.connect(self.reader_thread.deleteLater)
        self.reader_thread.start()
        print("[GUI] Hilo lector de estado iniciado.")

    def poll_status(self):
        if not self.is_homing:
            spi_send_three(0, 0, 0)

    def handle_status(self, status: str):
        if status == "501" and self.is_homing:
            print("[GUI] ¡Recibido 501! Homing completado.")
            self.log.appendPlainText("INFO: Puesta a 0 completada.")
            self.is_homing = False
            self.btn_home.setText("Home")
            self.btn_home.setEnabled(True)
            self.show_homing_complete_popup()
        elif status == "500":
            if not self.is_homing:
                self.is_homing = True
                self.btn_home.setText("Puesta a 0...")
                self.btn_home.setEnabled(False)

    def show_homing_complete_popup(self):
        print("[GUI] Mostrando pop-up de 'Puesta a 0 completada'")
        msg_box = QtWidgets.QMessageBox(self)
        try:
            icon_pixmap = self.style().standardPixmap(QtWidgets.QStyle.SP_DialogApplyButton)
            msg_box.setIconPixmap(icon_pixmap.scaled(64, 64))
        except Exception:
            msg_box.setIcon(QtWidgets.QMessageBox.Information)
        msg_box.setWindowTitle("Proceso Finalizado")
        msg_box.setText("Puesta a 0 completada")
        msg_box.setStandardButtons(QtWidgets.QMessageBox.Ok)
        run_dialog(msg_box)

    def closeEvent(self, event):
        print("[GUI] Cerrando aplicación, deteniendo hilos...")
        self.poll_timer.stop()
        if hasattr(self, 'status_reader'):
            self.status_reader.stop()
        if hasattr(self, 'reader_thread'):
            self.reader_thread.quit()
            if not self.reader_thread.wait(1000):
                print("[GUI] Hilo lector no responde, terminando...")
                self.reader_thread.terminate()
        event.accept()

def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    try:
        if hasattr(QtGui, "QGuiApplication") and hasattr(QtCore.Qt, "HighDpiScaleFactorRoundingPolicy"):
            QtGui.QGuiApplication.setHighDpiScaleFactorRoundingPolicy(
                QtCore.Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
            )
    except Exception:
        pass
    win = RobotUI()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
