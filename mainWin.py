import sys
from mainWinUI import Ui_MainWindow
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtGui import QIcon

class  configPage(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(configPage, self).__init__()
        self.setupUi(self)
        self.setWindowTitle('FMC205基带射频配置')
        self.setWindowIcon(QIcon('rss1.ico'))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    mainWin = configPage()
    mainWin.show()
    sys.exit(app.exec_())