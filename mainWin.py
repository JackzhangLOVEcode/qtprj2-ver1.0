import sys
from mainWinUI import Ui_MainWindow
from PyQt5.QtWidgets import QApplication, QMainWindow

if __name__ == "__main__":
    app = QApplication(sys.argv)
    mainWin = QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(mainWin)
    mainWin.show()
    sys.exit(app.exec_())