import sys
from dataclasses import dataclass

# ==== Compatibilidad con distintos bindings Qt ====
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

def run_dialog(dlg: QtWidgets.QDialog) -> int:
    """Usa exec() o exec_() según el binding disponible."""
    return dlg.exec() if hasattr(dlg, "exec") else dlg.exec_()

# ==== Modelo de comando (adaptá a tu protocolo real) ====
@dataclass
class Command:
    type: str
    payload: dict

# ==== Diálogo: MODO MANUAL (dial 0–100 + selección de eje) ====
class ManualDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Modo manual")
        self.setModal(True)

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(14, 14, 14, 14)
        lay.setSpacing(10)

        self.info = QtWidgets.QLabel("Girá el dial para habilitar y ajustar el nivel (0–100).")
        self.info.setWordWrap(True)

        # Selector de eje 1..5
        axis_row = QtWidgets.QHBoxLayout()
        axis_row.addWidget(QtWidgets.QLabel("Eje a mover:"))
        self.axis_combo = QtWidgets.QComboBox()
        self.axis_combo.addItems([f"Eje {i}" for i in range(1, 6)])
        axis_row.addWidget(self.axis_combo)
        axis_box = QtWidgets.QGroupBox("Selección de eje")
        axis_box.setLayout(axis_row)

        # Dial analógico
        self.dial = QtWidgets.QDial()
        self.dial.setRange(0, 100)
        self.dial.setNotchesVisible(True)
        self.dial.setValue(0)

        self.value_lbl = QtWidgets.QLabel("0")
        self.value_lbl.setAlignment(QtCore.Qt.AlignCenter)
        f = self.value_lbl.font()
        f.setPointSize(f.pointSize() + 4)
        f.setBold(True)
        self.value_lbl.setFont(f)

        btns = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)

        lay.addWidget(self.info)
        lay.addWidget(axis_box)
        lay.addWidget(self.dial)
        lay.addWidget(self.value_lbl)
        lay.addWidget(btns)

        self.dial.valueChanged.connect(lambda v: self.value_lbl.setText(str(v)))
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)

    def value(self) -> int:
        return int(self.dial.value())

    def axis(self) -> int:
        return self.axis_combo.currentIndex() + 1  # 1..5

# ==== Diálogo: VELOCIDAD (slider 0–100) ====
class SpeedDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Velocidad")
        self.setModal(True)

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(14, 14, 14, 14)
        lay.setSpacing(10)

        self.lbl = QtWidgets.QLabel("Seleccioná la velocidad (0–100).")

        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(0, 100)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(5)

        self.spin = QtWidgets.QSpinBox()
        self.spin.setRange(0, 100)

        self.slider.valueChanged.connect(self.spin.setValue)
        self.spin.valueChanged.connect(self.slider.setValue)
        self.slider.setValue(50)

        btns = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)

        lay.addWidget(self.lbl)
        lay.addWidget(self.slider)
        lay.addWidget(self.spin)
        lay.addWidget(btns)

        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)

    def value(self) -> int:
        return int(self.spin.value())

# ==== Ventana principal ====
class RobotUI(QtWidgets.QMainWindow):
    HOLD_INTERVAL_MS = 120  # envío continuo cada 120ms mientras se mantiene presionado

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Arm Controller")
        self.setMinimumSize(980, 560)

        # Estado del toggle Stop/Play
        # False = STOP (botón rojo ■). True = en stop (muestra PLAY verde ▶ para reanudar)
        self.is_stopped = False

        # Flechas naranjas GRANDES
        self.btn_up    = self._make_dir_button("▲", "Arriba", orange=True)
        self.btn_down  = self._make_dir_button("▼", "Abajo",  orange=True)
        self.btn_left  = self._make_dir_button("◀", "Izquierda", orange=True)
        self.btn_right = self._make_dir_button("▶", "Derecha",   orange=True)

        # Centro: Stop/Play (toggle con color) + abajo: PAUSE
        self.btn_stop_play = self._make_dir_button("■", "Stop / Play (Espacio)", orange=False)
        self._apply_stop_style()  # inicia en STOP (rojo)
        self.btn_pause = self._make_dir_button("PAUSE", "Pausa", orange=False)

        # Botones circulares (columna a la derecha del pad)
        self.btn_manual = self._make_round_button("MM", "MODO MANUAL")
        self.btn_speed  = self._make_round_button("VEL", "VELOCIDAD")

        # Entradas XYZ (columna derecha)
        self.x_in = self._make_spin("X (mm)", -10000, 10000, 0.1, 0.0)
        self.y_in = self._make_spin("Y (mm)", -10000, 10000, 0.1, 0.0)
        self.z_in = self._make_spin("Z (mm)", -10000, 10000, 0.1, 0.0)

        # Consola y acciones (abajo)
        self.btn_move_abs = QtWidgets.QPushButton("Move To (X, Y, Z)")
        self.btn_home     = QtWidgets.QPushButton("Home")
        self.log          = QtWidgets.QPlainTextEdit(readOnly=True)
        self.log.setPlaceholderText("Log de comandos…")

        # ====== Layout raíz: [ PAD | CIRCULARES | ENTRADAS ] ======
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QGridLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setHorizontalSpacing(14)
        root.setVerticalSpacing(12)

        # Columna 0: PAD (flechas + stop/play + pause)
        pad = QtWidgets.QGridLayout()
        pad.setHorizontalSpacing(10)
        pad.setVerticalSpacing(10)
        pad.addWidget(self.btn_up,        0, 1)
        pad.addWidget(self.btn_left,      1, 0)
        pad.addWidget(self.btn_stop_play, 1, 1)  # centro
        pad.addWidget(self.btn_right,     1, 2)
        pad.addWidget(self.btn_down,      2, 1)
        pad.addWidget(self.btn_pause,     3, 1)  # misma columna, debajo
        pad_box = self._group("Direcciones", pad)
        root.addWidget(pad_box, 0, 0)

        # Columna 1: Botones circulares pequeños
        col_circ = QtWidgets.QVBoxLayout()
        col_circ.setSpacing(12)
        col_circ.addWidget(self.btn_manual, alignment=QtCore.Qt.AlignTop | QtCore.Qt.AlignHCenter)
        col_circ.addWidget(self.btn_speed,  alignment=QtCore.Qt.AlignTop | QtCore.Qt.AlignHCenter)
        col_circ.addStretch(1)
        circ_box = self._group("Controles", col_circ)
        circ_box.setMaximumWidth(200)
        root.addWidget(circ_box, 0, 1)

        # Columna 2: Entradas XYZ
        form = QtWidgets.QFormLayout()
        form.addRow(self.x_in["label"], self.x_in["spin"])
        form.addRow(self.y_in["label"], self.y_in["spin"])
        form.addRow(self.z_in["label"], self.z_in["spin"])
        form_box = self._group("Entradas", form)
        root.addWidget(form_box, 0, 2)

        # Fila 1: Acciones
        actions = QtWidgets.QHBoxLayout()
        actions.addWidget(self.btn_move_abs)
        actions.addWidget(self.btn_home)
        act_box = self._group("Acciones", actions)
        root.addWidget(act_box, 1, 0, 1, 3)

        # Fila 2: Consola
        log_box = self._group("Consola", QtWidgets.QVBoxLayout())
        log_box.layout().addWidget(self.log)
        root.addWidget(log_box, 2, 0, 1, 3)

        # >>> Timer para envío continuo mientras está presionado <<<
        self._hold_timer = QtCore.QTimer(self)
        self._hold_timer.setInterval(self.HOLD_INTERVAL_MS)
        self._hold_timer.timeout.connect(self._emit_hold_while_pressed)
        self._held_dir: str | None = None  # "up","down","left","right" o None

        # Conexiones y atajos
        self._wire_buttons()
        self._install_shortcuts()

        # Ajustes mínimos
        for b in (self.btn_up, self.btn_down, self.btn_left, self.btn_right,
                  self.btn_stop_play, self.btn_pause, self.btn_move_abs, self.btn_home):
            b.setMinimumHeight(60)
        for spin in (self.x_in, self.y_in, self.z_in):
            spin["spin"].setMinimumWidth(120)

    # ---------- Helpers UI ----------
    def _group(self, title, inner_layout):
        box = QtWidgets.QGroupBox(title)
        box.setLayout(inner_layout)
        return box

    def _make_dir_button(self, text, tooltip, orange=False):
        """Botón grande (flechas / stop-play / pause)."""
        btn = QtWidgets.QPushButton(text)
        btn.setToolTip(tooltip)
        btn.setMinimumSize(100, 100)  # flechas grandes
        f = btn.font()
        f.setPointSize(f.pointSize() + 6)
        f.setBold(True)
        btn.setFont(f)

        base = """
            QPushButton { padding: 10px 14px; border: none; border-radius: 10px; }
            QPushButton:pressed { transform: scale(0.98); }
        """
        if orange:
            base += """
                QPushButton { background-color: #ff9800; color: #fff; }
                QPushButton:hover  { background-color: #fb8c00; }
                QPushButton:pressed{ background-color: #f57c00; }
            """
        else:
            base += """
                QPushButton { background-color: #e0e0e0; color: #222; }
                QPushButton:hover  { background-color: #d5d5d5; }
                QPushButton:pressed{ background-color: #cfcfcf; }
            """
        btn.setStyleSheet(base)
        return btn

    def _apply_stop_style(self):
        """Actualiza texto/tooltip/colores del botón central (STOP rojo / PLAY verde)."""
        if self.is_stopped:
            # Mostrar ▶ (verde) para reanudar
            self.btn_stop_play.setText("▶")
            self.btn_stop_play.setToolTip("Play / Reanudar (Espacio)")
            self.btn_stop_play.setStyleSheet("""
                QPushButton {
                    background-color: #2e7d32; color: #fff; border: none;
                    padding: 10px 14px; border-radius: 10px; font-weight: 700;
                }
                QPushButton:hover  { background-color: #388e3c; }
                QPushButton:pressed{ background-color: #1b5e20; }
            """)
        else:
            # Mostrar ■ (rojo) para detener
            self.btn_stop_play.setText("■")
            self.btn_stop_play.setToolTip("Stop (Espacio)")
            self.btn_stop_play.setStyleSheet("""
                QPushButton {
                    background-color: #c62828; color: #fff; border: none;
                    padding: 10px 14px; border-radius: 10px; font-weight: 700;
                }
                QPushButton:hover  { background-color: #d32f2f; }
                QPushButton:pressed{ background-color: #b71c1c; }
            """)

    def _make_round_button(self, text: str, tooltip: str):
        """Botón circular pequeño (64×64) para MM/Vel."""
        btn = QtWidgets.QPushButton(text)
        btn.setToolTip(tooltip)
        size = 64
        btn.setFixedSize(size, size)
        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: #e0e0e0; color: #333;
                border-radius: {size//2}px; border: 2px solid #9e9e9e;
                font-weight: 700;
            }}
            QPushButton:hover  {{ background-color: #d5d5d5; }}
            QPushButton:pressed{{ background-color: #cfcfcf; }}
        """)
        return btn

    def _make_spin(self, label, mn, mx, step, value):
        spin = QtWidgets.QDoubleSpinBox()
        spin.setRange(mn, mx)
        spin.setDecimals(3 if step < 1 else 1)
        spin.setSingleStep(step)
        spin.setValue(value)
        spin.setAlignment(QtCore.Qt.AlignRight)
        lab = QtWidgets.QLabel(label)
        return {"label": lab, "spin": spin}

    # ---------- Atajos ----------
    def _install_shortcuts(self):
        # Nota: los atajos siguen siendo de un “tick” (una vez).
        # Si querés repetición por teclado, puedo agregar keyPressEvent/keyRelease.
        QtWidgets.QShortcut(QtGui.QKeySequence("Up"),    self, activated=lambda: self._send_dir_once("up"))
        QtWidgets.QShortcut(QtGui.QKeySequence("Down"),  self, activated=lambda: self._send_dir_once("down"))
        QtWidgets.QShortcut(QtGui.QKeySequence("Left"),  self, activated=lambda: self._send_dir_once("left"))
        QtWidgets.QShortcut(QtGui.QKeySequence("Right"), self, activated=lambda: self._send_dir_once("right"))
        QtWidgets.QShortcut(QtGui.QKeySequence("Space"), self, activated=self._toggle_stop_play)
        QtWidgets.QShortcut(QtGui.QKeySequence("Return"), self, activated=self._move_abs)
        QtWidgets.QShortcut(QtGui.QKeySequence("Enter"),  self, activated=self._move_abs)
        QtWidgets.QShortcut(QtGui.QKeySequence("H"),      self, activated=self._home)

    # ---------- Conexiones ----------
    def _wire_buttons(self):
        # Flechas: envío CONTINUO mientras se mantiene presionado
        self.btn_up.pressed.connect(lambda: self._start_hold("up"))
        self.btn_down.pressed.connect(lambda: self._start_hold("down"))
        self.btn_left.pressed.connect(lambda: self._start_hold("left"))
        self.btn_right.pressed.connect(lambda: self._start_hold("right"))

        self.btn_up.released.connect(self._stop_hold)
        self.btn_down.released.connect(self._stop_hold)
        self.btn_left.released.connect(self._stop_hold)
        self.btn_right.released.connect(self._stop_hold)

        # Stop/Play y Pause (click normal)
        self.btn_stop_play.clicked.connect(self._toggle_stop_play)
        self.btn_pause.clicked.connect(self._pause)

        # Botones circulares -> diálogos
        self.btn_manual.clicked.connect(self._open_manual_dialog)
        self.btn_speed.clicked.connect(self._open_speed_dialog)

        # Acciones
        self.btn_move_abs.clicked.connect(self._move_abs)
        self.btn_home.clicked.connect(self._home)

    # ---------- Lógica de envío (hold) ----------
    def _start_hold(self, direction: str):
        """Arranca el envío continuo en la dirección dada."""
        self._held_dir = direction
        self._send_dir_once(direction)   # primer envío inmediato
        self._hold_timer.start()

    def _stop_hold(self):
        """Detiene el envío continuo."""
        self._held_dir = None
        self._hold_timer.stop()

    def _emit_hold_while_pressed(self):
        """Tick del timer: si hay una dirección en hold, envía otro comando."""
        if self._held_dir:
            self._send_dir_once(self._held_dir)
        else:
            self._hold_timer.stop()

    def _send_dir_once(self, direction: str):
        """Envía un 'nudge' de una sola vez en la dirección dada."""
        self.send_command(Command("nudge", {"dir": direction}))

    # ---------- Lógica de otros controles ----------
    def _toggle_stop_play(self):
        if not self.is_stopped:
            self.send_command(Command("stop", {}))
            self.is_stopped = True
        else:
            self.send_command(Command("play", {}))
            self.is_stopped = False
        self._apply_stop_style()

    def _pause(self):
        self.send_command(Command("pause", {}))

    def _move_abs(self):
        x = float(self.x_in["spin"].value())
        y = float(self.y_in["spin"].value())
        z = float(self.z_in["spin"].value())
        self.send_command(Command("move_abs", {"x": x, "y": y, "z": z}))

    def _home(self):
        self.send_command(Command("home", {}))

    def _open_manual_dialog(self):
        dlg = ManualDialog(self)
        if run_dialog(dlg) == QtWidgets.QDialog.Accepted:
            val = dlg.value()
            axis = dlg.axis()
            enabled = 1 if val > 0 else 0
            self.send_command(Command("manual_enable", {"enabled": enabled, "level": val, "axis": axis}))

    def _open_speed_dialog(self):
        dlg = SpeedDialog(self)
        if run_dialog(dlg) == QtWidgets.QDialog.Accepted:
            val = dlg.value()
            self.send_command(Command("set_speed_pct", {"value": val}))

    # ---------- Transporte / Log ----------
    def send_command(self, cmd: Command):
        t = cmd.type
        p = cmd.payload
        if t == "nudge":
            line = f"NUDGE,{p['dir']}"
        elif t == "move_abs":
            line = f"MOVE,{p['x']:.3f},{p['y']:.3f},{p['z']:.3f}"
        elif t == "home":
            line = "HOME"
        elif t == "stop":
            line = "STOP"
        elif t == "play":
            line = "PLAY"
        elif t == "pause":
            line = "PAUSE"
        elif t == "manual_enable":
            line = f"MANUAL,{p['enabled']},{p['level']},{p.get('axis',1)}"
        elif t == "set_speed_pct":
            line = f"SPEED_PCT,{p['value']}"
        else:
            line = f"UNKNOWN,{t}"
        self._log_line(line)

    def _log_line(self, text: str):
        self.log.appendPlainText(text)
        c = self.log.textCursor()
        c.movePosition(QtGui.QTextCursor.End)
        self.log.setTextCursor(c)

# ==== Main ====
def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")  # quitá si querés look nativo
    win = RobotUI()
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
