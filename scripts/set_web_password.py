#!/usr/bin/env python3
"""Configure a fixed local credential interactively; never print or randomize it."""
import getpass
import os
from pathlib import Path

if __name__ == '__main__':
    password = getpass.getpass('设置固定网页密码（8 至 256 字符）：')
    if not 8 <= len(password) <= 256 or password != password.strip():
        raise SystemExit('密码长度不符，或开头结尾存在空白。')
    if password != getpass.getpass('再次输入：'):
        raise SystemExit('两次密码不一致。')
    root = Path(__file__).resolve().parents[1] / '.runtime/remote_web'
    root.mkdir(parents=True, mode=0o700, exist_ok=True)
    root.chmod(0o700)
    path = root / 'password.txt'
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, 'w') as file:
        os.fchmod(file.fileno(), 0o600)
        file.write(password + '\n')
    print('固定密码已保存。网页正在运行时，重启网页服务后生效。')
