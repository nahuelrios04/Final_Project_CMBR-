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
def spi_send_three(cmd: int, arg1: int, arg2: int):
    """Envía 3 enteros al bridge C que controla el SPI."""
    try:
        # Formato: I (int) cmd arg1 arg2
        with open(FIFO_PATH, "a") as f:
            f.write(f"I {int(cmd)} {int(arg1)} {int(arg2)}\n")
    except Exception as e:
        if cmd != 0: # Ignorar errores en polling
            print(f"[SPI-FIFO] Error escritura: {e}")

# ==== LÓGICA DE JOGGING (Teclado/Flechas Main) ====
# Mapeo: Eje 1 = 601 (Izq/Der), Eje 2 = 602 (Arr/Abj)
# Args: 0=Stop, 1=CW (Positivo), 2=CCW (Negativo)

def jog_start(direction: str):
    cmd = 0
    arg1 = 0
    
    if direction == "left":
        cmd = 601; arg1 = 2 # Eje 1 CCW
    elif direction == "right":
        cmd = 601; arg1 = 1 # Eje 1 CW
    elif direction == "up":
        cmd = 602; arg1 = 1 # Eje 2 CW (Asumiendo arriba=positivo)
    elif direction == "down":
        cmd = 602; arg1 = 2 # Eje 2 CCW
        
    if cmd != 0:
        spi_send_three(cmd, arg1, 0)

def jog_stop():
    # Por seguridad enviamos STOP a los dos ejes principales
    spi_send_three(601, 0, 0) # Stop Eje 1
    spi_send_three(602, 0, 0) # Stop Eje 2

# ==== sizing ====
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
    
    sizes = {
        "U":U, "btn_big":int(U*1.45), "btn_small":int(U*0.95),
        "pad":int(U*0.26), "radius":int(U*0.22),
        "font_delta_big":2, "font_delta_norm":1,
    }
    # Clamps
    sizes["btn_big"] = max(54, min(sizes["btn_big"], 120))
    sizes["btn_small"] = max(38, min(sizes["btn_small"], 86))
    sizes["pad"] = max(6, min(sizes["pad"], 16))
    sizes["radius"] = max(6, min(sizes["radius"], 12))
    return sizes

# ==== Diálogos ====

class ManualDialog(QtWidgets.QDialog):
    """
    Control Manual Específico:
    Envía comandos 601, 602, 603 directamente al pulsar los botones.
    """
    def __init__(self, parent=None, sender_callback=None):
        super().__init__(parent)
        self.setWindowTitle("Modo Manual (Jogging)")
        self.setModal(True)
        self.sender = sender_callback # Función para enviar SPI directamente
        S = compute_units(self)
        
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(S["pad"], S["pad"], S["pad"], S["pad"])
        
        # --- Selección de Eje ---
        row_axis = QtWidgets.QHBoxLayout()
        row_axis.addWidget(QtWidgets.QLabel("Eje a controlar:"))
        self.combo_axis = QtWidgets.QComboBox()
        self.combo_axis.addItems(["Eje 1 (Base)", "Eje 2 (Brazo)", "Eje 3 (Elevador)"])
        row_axis.addWidget(self.combo_axis)
        lay.addLayout(row_axis)
        
        # --- Botones de Movimiento ---
        btn_layout = QtWidgets.QHBoxLayout()
        
        # Estilos
        base_style = f"padding:{S['pad']}px; border-radius:{S['radius']}px; font-weight:bold; min-width:{S['btn_big']}px; min-height:{S['btn_big']}px;"
        style_orange = base_style + "background-color:#ff9800; color:white;"
        style_red = base_style + "background-color:#d32f2f; color:white;"
        
        self.btn_ccw = QtWidgets.QPushButton("◀ (CCW / -)")
        self.btn_ccw.setStyleSheet(style_orange)
        
        self.btn_stop = QtWidgets.QPushButton("■ STOP")
        self.btn_stop.setStyleSheet(style_red)
        
        self.btn_cw = QtWidgets.QPushButton("▶ (CW / +)")
        self.btn_cw.setStyleSheet(style_orange)
        
        btn_layout.addWidget(self.btn_ccw)
        btn_layout.addWidget(self.btn_stop)
        btn_layout.addWidget(self.btn_cw)
        lay.addLayout(btn_layout)
        
        # --- Estado ---
        self.lbl_status = QtWidgets.QLabel("Listo. Mantén presionado o pulsa.")
        self.lbl_status.setAlignment(QtCore.Qt.AlignCenter)
        lay.addWidget(self.lbl_status)
        
        btns = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Close)
        btns.rejected.connect(self.reject)
        lay.addWidget(btns)

        # --- Conexiones (Lógica Instantánea) ---
        # Al presionar: Enviar movimiento
        # Al soltar: Enviar stop
        self.btn_ccw.pressed.connect(lambda: self._send_move(2)) # 2 = CCW
        self.btn_ccw.released.connect(lambda: self._send_move(0)) # 0 = Stop
        
        self.btn_cw.pressed.connect(lambda: self._send_move(1))  # 1 = CW
        self.btn_cw.released.connect(lambda: self._send_move(0))
        
        self.btn_stop.clicked.connect(lambda: self._send_move(0))

    def _send_move(self, direction_code):
        # Determinar comando base según eje (601, 602, 603)
        # index 0 -> 601, index 1 -> 602...
        base_cmd = 601 + self.combo_axis.currentIndex()
        
        status_txt = "STOP"
        if direction_code == 1: status_txt = "MOVIENDO CW (+)"
        if direction_code == 2: status_txt = "MOVIENDO CCW (-)"
        
        self.lbl_status.setText(f"Eje {self.combo_axis.currentIndex()+1}: {status_txt}")
        
        # Enviar al SPI
        if self.sender:
            self.sender(base_cmd, direction_code, 0)

# -----------------------------------------------------------------
# Hilo lector (Rx)
# -----------------------------------------------------------------
class StatusReader(QtCore.QObject):
    statusReceived = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        self.is_running = True

    def run(self):
        while self.is_running:
            try:
                if not os.path.exists(FIFO_RX_PATH):
                    time.sleep(1)
                    continue
                
                # Bloqueante hasta que haya datos
                with open(FIFO_RX_PATH, 'r') as fifo:
                    while self.is_running:
                        line = fifo.readline()
                        if line:
                            self.statusReceived.emit(line.strip())
                        else:
                            time.sleep(1)
                            break
            except Exception:
                time.sleep(1)
        self.finished.emit()

    def stop(self):
        self.is_running = False

# ==== Ventana principal ====
class RobotUI(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Arm Controller (SPI Adjusted)")
        self.S = compute_units(self)
        self.is_stopped = False
        self.current_speed = 50 

        # Botones Dirección
        self.btn_up = self._make_dir_button("▲", "Eje 2 Positivo (Arr)", True)
        self.btn_down = self._make_dir_button("▼", "Eje 2 Negativo (Abj)", True)
        self.btn_left = self._make_dir_button("◀", "Eje 1 Negativo (Izq)", True)
        self.btn_right = self._make_dir_button("▶", "Eje 1 Positivo (Der)", True)

        self.btn_stop_play = self._make_dir_button("■", "Stop (Deshabilita todo)", False)
        self._apply_stop_style()
        
        # Botones Control
        self.btn_manual = self._make_round_button("M", "Modo Manual Avanzado")
        self.btn_home = QtWidgets.QPushButton("HOME (Puesta a 0)")
        self.btn_home.setMinimumHeight(self.S["btn_big"])
        
        self.log = QtWidgets.QPlainTextEdit(readOnly=True)
        self.log.setPlaceholderText("Historial de comandos...")

        # Layout
        central = QtWidgets.QWidget(); self.setCentralWidget(central)
        root = QtWidgets.QGridLayout(central)
        root.setContentsMargins(10, 10, 10, 10)

        # Pad Direccional
        pad = QtWidgets.QGridLayout()
        pad.addWidget(self.btn_up, 0, 1)
        pad.addWidget(self.btn_left, 1, 0)
        pad.addWidget(self.btn_stop_play, 1, 1)
        pad.addWidget(self.btn_right, 1, 2)
        pad.addWidget(self.btn_down, 2, 1)
        
        # Columna Derecha
        right_col = QtWidgets.QVBoxLayout()
        right_col.addWidget(self.btn_manual)
        right_col.addWidget(self.btn_home)
        right_col.addStretch()

        root.addLayout(pad, 0, 0)
        root.addLayout(right_col, 0, 1)
        root.addWidget(self.log, 1, 0, 1, 2)

        self._wire_buttons()
        self._install_shortcuts()

        # Estado
        self.is_homing = False
        self.setup_status_reader_thread()
        
        # Polling timer (Ping SPI para ver si está vivo)
        self.poll_timer = QtCore.QTimer(self)
        self.poll_timer.timeout.connect(lambda: spi_send_three(0, 0, 0))
        self.poll_timer.start(500)

        self.resize(600, 400)

    def _make_dir_button(self, text, tooltip, orange=False):
        btn = QtWidgets.QPushButton(text)
        btn.setToolTip(tooltip)
        s = self.S["btn_big"]
        btn.setFixedSize(s, s)
        # Estilo simple
        color = "#ff9800" if orange else "#d32f2f"
        btn.setStyleSheet(f"background-color:{color}; color:white; font-weight:bold; border-radius:10px; font-size:18px;")
        return btn

    def _make_round_button(self, text, tooltip):
        btn = QtWidgets.QPushButton(text)
        btn.setToolTip(tooltip)
        s = self.S["btn_small"]
        btn.setFixedSize(s, s)
        btn.setStyleSheet(f"background-color:#2196f3; color:white; border-radius:{s//2}px; font-weight:bold;")
        return btn

    def _apply_stop_style(self):
        if self.is_stopped:
            self.btn_stop_play.setText("▶")
            self.btn_stop_play.setStyleSheet("background-color:#4caf50; color:white; border-radius:10px; font-size:18px;")
        else:
            self.btn_stop_play.setText("■")
            self.btn_stop_play.setStyleSheet("background-color:#d32f2f; color:white; border-radius:10px; font-size:18px;")

    def _install_shortcuts(self):
        QtWidgets.QShortcut(QtGui.QKeySequence("Left"), self, activated=lambda: jog_start("left"))
        QtWidgets.QShortcut(QtGui.QKeySequence("Right"), self, activated=lambda: jog_start("right"))
        QtWidgets.QShortcut(QtGui.QKeySequence("Up"), self, activated=lambda: jog_start("up"))
        QtWidgets.QShortcut(QtGui.QKeySequence("Down"), self, activated=lambda: jog_start("down"))
        QtWidgets.QShortcut(QtGui.QKeySequence("Space"), self, activated=self._toggle_stop)
        
        # Eventos de soltar tecla (Release) es difícil con QShortcut estándar, 
        # normalmente se usa keyReleaseEvent override, pero usamos botones UI principalmente.
        # Para teclado fluido se requeriría reimplementar keyPressEvent/keyReleaseEvent.

    def _wire_buttons(self):
        # Conectar Pressed/Released para movimiento fluido
        self.btn_left.pressed.connect(lambda: (jog_start("left"), self.log_cmd("JOG LEFT (Eje 1 CCW)")))
        self.btn_left.released.connect(jog_stop)
        
        self.btn_right.pressed.connect(lambda: (jog_start("right"), self.log_cmd("JOG RIGHT (Eje 1 CW)")))
        self.btn_right.released.connect(jog_stop)
        
        self.btn_up.pressed.connect(lambda: (jog_start("up"), self.log_cmd("JOG UP (Eje 2 CW)")))
        self.btn_up.released.connect(jog_stop)
        
        self.btn_down.pressed.connect(lambda: (jog_start("down"), self.log_cmd("JOG DOWN (Eje 2 CCW)")))
        self.btn_down.released.connect(jog_stop)
        
        self.btn_stop_play.clicked.connect(self._toggle_stop)
        self.btn_manual.clicked.connect(self._open_manual_dialog)
        self.btn_home.clicked.connect(self._start_homing)

    def _toggle_stop(self):
        self.is_stopped = not self.is_stopped
        self._apply_stop_style()
        if self.is_stopped:
            jog_stop() # Envía STOP a ejes
            self.log_cmd("STOP GLOBAL")
        else:
            self.log_cmd("SISTEMA HABILITADO")

    def _start_homing(self):
        if self.is_homing: return
        self.is_homing = True
        self.btn_home.setEnabled(False)
        self.btn_home.setText("Homing en curso...")
        self.log_cmd("INICIO HOMING (CMD 500)")
        spi_send_three(500, 0, 0)

    def _open_manual_dialog(self):
        # Pasamos la función spi_send_three para que el diálogo la use directamente
        dlg = ManualDialog(self, sender_callback=spi_send_three)
        run_dialog(dlg)

    def log_cmd(self, text):
        self.log.appendPlainText(f">> {text}")
        c = self.log.textCursor(); c.movePosition(QtGui.QTextCursor.End); self.log.setTextCursor(c)

    # --- Lector de Estado ---
    def setup_status_reader_thread(self):
        self.reader_thread = QtCore.QThread()
        self.status_reader = StatusReader()
        self.status_reader.moveToThread(self.reader_thread)
        self.reader_thread.started.connect(self.status_reader.run)
        self.status_reader.statusReceived.connect(self.handle_status)
        self.status_reader.finished.connect(self.reader_thread.quit)
        self.reader_thread.start()

    def handle_status(self, status):
        # Si el ESP32 envía algo interesante (ej: 501 al terminar homing)
        # Nota: Tu código de ESP32 no enviaba TX explícito más allá de eco, 
        # pero si añades lógica de respuesta, aquí se recibe.
        if "OK" in status: return # Ignorar ACK
        self.log_cmd(f"RX: {status}")
        
        # Simulación de fin de homing si recibes algo específico
        if status == "HOMING_DONE" or status == "501": 
            self.is_homing = False
            self.btn_home.setEnabled(True)
            self.btn_home.setText("HOME (Puesta a 0)")
            QtWidgets.QMessageBox.information(self, "Info", "Homing Completado")

    def closeEvent(self, event):
        self.poll_timer.stop()
        if hasattr(self, 'status_reader'): self.status_reader.stop()
        event.accept()

def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    win = RobotUI()
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
