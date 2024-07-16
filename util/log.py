#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@File    :   log.py
@Time    :   2024/03/08 12:42:35
@Author  :   George Zeng
@Contact :   george.zeng@syensqo.com
@Version :   1.0.0
"""


import logging
import logging.handlers
import os


class LogUtil:
    def __init__(self, project: str):
        self.logger = logging.getLogger("%s.logger" % project)
        self.project = project

    def logger_init(self):
        self.logger.setLevel(logging.INFO)

        if not os.path.exists('./log'):
            os.mkdir('./log')

        logfile = './log/%s.log' % self.project

        fh = logging.handlers.RotatingFileHandler(logfile, mode='a', maxBytes=1024 * 1024 * 50, backupCount=30)

        fh.setLevel(logging.INFO)

        ch = logging.StreamHandler()

        ch.setLevel(logging.INFO)

        formatter = logging.Formatter("%(asctime)s - %(levelname)s - %(module)s: %(message)s")

        fh.setFormatter(formatter)

        ch.setFormatter(formatter)

        self.logger.addHandler(fh)

        self.logger.addHandler(ch)
