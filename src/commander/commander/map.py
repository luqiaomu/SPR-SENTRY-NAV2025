import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QFileDialog, QLabel, QVBoxLayout
from PyQt5.QtGui import QPixmap

class ImageSelectorApp(QWidget):
    def __init__(self):
        super().__init__()

        self.init_ui()

    def init_ui(self):
        # 创建布局
        layout = QVBoxLayout()

        # 创建按钮
        self.select_button = QPushButton('选择图片', self)
        self.select_button.clicked.connect(self.show_dialog)

        # 创建标签用于显示图片
        self.image_label = QLabel(self)

        # 将控件添加到布局中
        layout.addWidget(self.select_button)
        layout.addWidget(self.image_label)

        # 设置主窗口布局
        self.setLayout(layout)

        # 设置主窗口属性
        self.setWindowTitle('Image Selector App')
        self.setGeometry(100, 100, 400, 300)

    def show_dialog(self):
        # 弹出文件选择对话框
        file_path, _ = QFileDialog.getOpenFileName(self, '选择图片', '')

        if file_path:
            # 显示选择的图片
            self.show_image(file_path)

    def show_image(self, image_path):
        # 显示图片
        pixmap = QPixmap(image_path)
        self.image_label.setPixmap(pixmap)
        self.image_label.setScaledContents(True)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ImageSelectorApp()
    window.show()
    sys.exit(app.exec_())
