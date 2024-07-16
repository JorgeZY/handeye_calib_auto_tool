#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@File    :   main.py
@Time    :   2024/03/08 18:27:01
@Author  :   George Zeng
@Contact :   george.zeng@syensqo.com
@Version :   1.0.0
"""

import sys
from dotenv import dotenv_values
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication
from qfluentwidgets import setTheme, Theme

from view.handeye_calib import HandEyeCalib


if __name__ == "__main__":
    QApplication.setHighDpiScaleFactorRoundingPolicy(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps)

    setTheme(Theme.AUTO)

    app = QApplication(sys.argv)

    env = dotenv_values(".env", verbose=True)
    win = HandEyeCalib(env)

    win.show()
    sys.exit(app.exec())