#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys


def read_pgm(path):
    with open(path, 'rb') as f:
        if f.readline().strip() != b'P5':
            raise ValueError('not P5')
        tokens = []
        while len(tokens) < 2:
            line = f.readline()
            if line.startswith(b'#'):
                continue
            tokens += line.split()
        w, h = int(tokens[0]), int(tokens[1])
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        data = f.read()
        return w, h, data


w, h, data = read_pgm(sys.argv[1])
n = w * h
# map_saver 规范：254=空闲(白) 0=障碍(黑) 205=未知(灰)
occ = sum(1 for b in data[:n] if b < 100)
free = sum(1 for b in data[:n] if b > 245)
unk = n - occ - free
print("尺寸: %dx%d (%.1f x %.1f 米)" % (w, h, w * 0.05, h * 0.05))
print("障碍(黑): %d (%.1f%%)" % (occ, 100.0 * occ / n))
print("空闲(白): %d (%.1f%%)" % (free, 100.0 * free / n))
print("未知(灰): %d (%.1f%%)" % (unk, 100.0 * unk / n))
