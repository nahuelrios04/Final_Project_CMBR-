import sys
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

# HiDPI
try:
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
except Exception:
    pass

FIFO_PATH = "/tmp/spi_tx"

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
        print(f"[SPI-FIFO] No pude escribir: {e}")

def jog_start(direction: str):
    # 210 = START_JOG; a1 = 3 (izq) / 4 (der) / (1 up / 2 down si querés)
    dir_map = {"left":3, "right":4, "up":1, "down":2}
    spi_send_three(210, dir_map.get(direction, 0), 0)

def jog_stop():
    # 211 = STOP_JOG
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
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Modo manual")
        self.setModal(True)
        S = compute_units(self)

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(S["pad"], S["pad"], S["pad"], S["pad"])
        lay.setSpacing(int(S["pad"]*0.7))

        self.info = QtWidgets.QLabel("Girá el dial para habilitar y ajustar el nivel (0–100).")
        self.info.setWordWrap(True)

        axis_row = QtWidgets.QHBoxLayout()
        axis_row.addWidget(QtWidgets.QLabel("Eje a mover:"))
        self.axis_combo = QtWidgets.QComboBox()
        self.axis_combo.addItems([f"Eje {i}" for i in range(1,6)])
        axis_row.addWidget(self.axis_combo)
        axis_box = QtWidgets.QGroupBox("Selección de eje")
        axis_box.setLayout(axis_row)

        self.dial = QtWidgets.QDial()
        self.dial.setRange(0,100)
        self.dial.setNotchesVisible(True)
        self.dial.setValue(0)

        self.value_lbl = QtWidgets.QLabel("0")
        self.value_lbl.setAlignment(QtCore.Qt.AlignCenter)
        f = self.value_lbl.font(); f.setPointSize(f.pointSize()+S["font_delta_big"]); f.setBold(True)
        self.value_lbl.setFont(f)

        btns = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok|QtWidgets.QDialogButtonBox.Cancel)

        lay.addWidget(self.info); lay.addWidget(axis_box); lay.addWidget(self.dial)
        lay.addWidget(self.value_lbl); lay.addWidget(btns)

        self.dial.valueChanged.connect(lambda v: self.value_lbl.setText(str(v)))
        btns.accepted.connect(self.accept); btns.rejected.connect(self.reject)

    def value(self) -> int: return int(self.dial.value())
    def axis(self) -> int:  return self.axis_combo.currentIndex()+1

class SpeedDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
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
        self.slider.setValue(50)

        btns = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok|QtWidgets.QDialogButtonBox.Cancel)
        lay.addWidget(self.lbl); lay.addWidget(self.slider); lay.addWidget(self.spin); lay.addWidget(btns)
        btns.accepted.connect(self.accept); btns.rejected.connect(self.reject)

    def value(self) -> int: return int(self.spin.value())

# ==== Ventana principal ====
class RobotUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Arm Controller")
        self.S = compute_units(self)
        self.is_stopped = False

        self.btn_up    = self._make_dir_button("▲", "Arriba", True)
        self.btn_down  = self._make_dir_button("▼", "Abajo",  True)
        self.btn_left  = self._make_dir_button("◀", "Izquierda", True)
        self.btn_right = self._make_dir_button("▶", "Derecha",   True)

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

        for b in (self.btn_up, self.btn_down, self.btn_left, self.btn_right,
                  self.btn_stop_play, self.btn_pause, self.btn_move_abs, self.btn_home):
            b.setMinimumHeight(int(self.S["btn_big"]*0.55))
        for spin in (self.x_in, self.y_in, self.z_in):
            spin["spin"].setMinimumWidth(int(self.S["btn_small"]*1.8))

        try: self.showMaximized()
        except Exception: self.resize(980,560)

    # UI helpers
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
                 "QPushButton:pressed { transform: scale(0.98);} " )
        if orange:
            base += "QPushButton { background-color:#ff9800; color:#fff; }" \
                    "QPushButton:hover{ background-color:#fb8c00; }" \
                    "QPushButton:pressed{ background-color:#f57c00; }"
        else:
            base += "QPushButton { background-color:#e0e0e0; color:#222; }" \
                    "QPushButton:hover{ background-color:#d5d5d5; }" \
                    "QPushButton:pressed{ background-color:#cfcfcf; }"
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

    # Shortcuts
    def _install_shortcuts(self):
        QtWidgets.QShortcut(QtGui.QKeySequence("Left"),  self, activated=lambda: jog_start("left"))
        QtWidgets.QShortcut(QtGui.QKeySequence("Right"), self, activated=lambda: jog_start("right"))
        QtWidgets.QShortcut(QtGui.QKeySequence("Space"), self, activated=self._toggle_stop_play)

    # Wiring (con JOG continuo en flechas)
    def _wire_buttons(self):
        self.btn_left.pressed.connect(lambda: (jog_start("left"),  self.send_command(Command("nudge", {"dir": "left"}))))
        self.btn_left.released.connect(jog_stop)
        self.btn_right.pressed.connect(lambda: (jog_start("right"), self.send_command(Command("nudge", {"dir": "right"}))))
        self.btn_right.released.connect(jog_stop)
        # (opcional up/down también:)
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

    # Acciones que loguean (y algunas mandan extra por SPI si querés)
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
        spi_send_three(400, int(x*1000), int(y*1000))  # ejemplo: X,Y en milésimas

    def _home(self):
        self.send_command(Command("home", {})); spi_send_three(500, 0, 0)

    def _open_manual_dialog(self):
        dlg = ManualDialog(self)
        if run_dialog(dlg) == QtWidgets.QDialog.Accepted:
            val = dlg.value(); axis = dlg.axis(); enabled = 1 if val>0 else 0
            self.send_command(Command("manual_enable", {"enabled":enabled, "level":val, "axis":axis}))
            spi_send_three(600, axis, val)

    def _open_speed_dialog(self):
        dlg = SpeedDialog(self)
        if run_dialog(dlg) == QtWidgets.QDialog.Accepted:
            val = dlg.value()
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
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
