"""ANSI escape code utilities."""
from __future__ import annotations

import html
import re

SGR_RE = re.compile(r'\x1b\[((?:\d+;)*\d*)m')
OSC_SEQ_RE = re.compile(r'\x1b\][^\x07\x1b]*(?:\x07|\x1b\\)')
CSI_SEQ_RE = re.compile(r'\x1b\[[0-9;?]*[ -/]*[@-~]')

COLOR_MAP = {
    30: '#000000', 31: '#ff5555', 32: '#50fa7b', 33: '#f1fa8c',
    34: '#bd93f9', 35: '#ff79c6', 36: '#8be9fd', 37: '#bbbbbb',
    90: '#666666', 91: '#ff6e6e', 92: '#69ff94', 93: '#ffffa5',
    94: '#d6acff', 95: '#ff92df', 96: '#a4ffff', 97: '#ffffff'
}


def ansi_to_html(chunk: str) -> str:
    """Convert ANSI escape sequences to HTML."""
    text = html.escape(chunk)
    span_stack, out = [], []
    i = 0
    for m in SGR_RE.finditer(text):
        out.append(text[i:m.start()])
        params = m.group(1) or '0'
        if params == '0':
            while span_stack:
                out.append('</span>')
                span_stack.pop()
            i = m.end()
            continue
        styles = []
        for p in params.split(';'):
            try:
                code = int(p)
            except ValueError:
                continue
            if code == 1:
                styles.append('font-weight:bold')
            elif code in COLOR_MAP:
                styles.append(f'color:{COLOR_MAP[code]}')
            elif code == 39:
                styles.append('color:#ffffff')
            elif code == 22:
                styles.append('font-weight:normal')
        if styles:
            out.append(f"<span style=\"{' ;'.join(styles)}\">")
            span_stack.append('</span>')
        i = m.end()
    out.append(text[i:])
    while span_stack:
        out.append(span_stack.pop())
    res = ''.join(out)
    return res.replace('\r\n', '\n').replace('\r', '\n').replace('\n', '<br>')


__all__ = ['ansi_to_html', 'CSI_SEQ_RE', 'OSC_SEQ_RE']
