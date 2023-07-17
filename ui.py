import sys
import subprocess
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QLineEdit, QPushButton

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GitHub Repository Cloner")
        self.setGeometry(300, 300, 400, 200)

        self.label = QLabel("Enter the repository URL:", self)
        self.label.setGeometry(50, 50, 300, 20)

        self.textbox = QLineEdit(self)
        self.textbox.setGeometry(50, 80, 300, 20)

        self.button = QPushButton("Clone Repository", self)
        self.button.setGeometry(150, 120, 100, 30)
        self.button.clicked.connect(self.clone_repo)

        self.result_label = QLabel(self)
        self.result_label.setGeometry(50, 160, 300, 20)

    def clone_repo(self):
        repo_url = self.textbox.text()

        # Run git clone command
        process = subprocess.Popen(["git", "clone", repo_url], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        output, error = process.communicate()

        if process.returncode == 0:
            self.result_label.setText("Repository cloned successfully!")
        else:
            self.result_label.setText("Error cloning repository. Please check the URL and try again.")
            print(error.decode())

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
