#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import sys, ujson
import matplotlib.pyplot as plt
import numpy as np
from functools import partial


from sklearn.metrics import confusion_matrix, accuracy_score, precision_recall_fscore_support #, classification_report
import scipy
from .constants import KEY_CODES

def _get(o, p):
    if isinstance(p, str): p = p.split('.')
    for i in p: o = o[i]
    return o

# http://code.activestate.com/recipes/577452-a-memoize-decorator-for-instance-methods/
class memoize(object):
    def __init__(self, func):
        self.func = func
    def __get__(self, obj, objtype=None):
        if obj is None:
            return self.func
        return partial(self, obj)
    def __call__(self, *args, **kw):
        obj = args[0]
        try:
            cache = obj.__cache
        except AttributeError:
            cache = obj.__cache = {}
        key = (self.func, args[1:], frozenset(kw.items()))
        try:
            res = cache[key]
        except KeyError:
            res = cache[key] = self.func(*args, **kw)
        return res

class Stats(object):
    def __init__(self, config, detailed=True):
        self.config = config

        self.live_plot = False

        self.test_size = 0
        self.training_cost = np.ndarray(shape=(0,), dtype=float)
        self.actual = np.ndarray(shape=(0,self.test_size), dtype=int)
        self.predicted = np.ndarray(shape=(0,self.test_size), dtype=int)
        self.testing_cost = np.ndarray(shape=(0,), dtype=float)

        self.count = 0

        self.plots = [
                ('accuracy', dict(color='C0', lw=2, label="Accuracy")),
                # ('training_cost', dict(color='C1', lw=2, label="Training cost")),
                # ('testing_cost', dict(color='C2', lw=2, label="Testing cost")),
        ]

        gray = (0, 0, 0, 0.3)
        if detailed:
# 20, 34, 48, 21, 35, 49, 22, 36, 50, 57
# T   G   B   Y   H   N   U   J   M   _

            self.plots += [
                (('keys', 20, 'fscore'), dict(color=gray, ls='dashed', lw=1, label='F-Score T, G, B, Y, U, M')),
                (('keys', 34, 'fscore'), dict(color=gray, ls='dashed', lw=1, label='F-Score T, G, B, Y, U, M')),
                (('keys', 48, 'fscore'), dict(color=gray, ls='dashed', lw=1, label='F-Score T, G, B, Y, U, M')),
                (('keys', 21, 'fscore'), dict(color=gray, ls='dashed', lw=1, label='F-Score T, G, B, Y, U, M')),
                (('keys', 22, 'fscore'), dict(color=gray, ls='dashed', lw=1, label='F-Score T, G, B, Y, U, M')),
                (('keys', 50, 'fscore'), dict(color=gray, ls='dashed', lw=1, label='F-Score T, G, B, Y, U, M')),
                (('keys', 49, 'fscore'), dict(color=gray, lw=2, label='F-Score N')),

                (('keys', 35, 'fscore'), dict(color='C1', lw=1, label='F-Score H')),
                (('keys', 0, 'fscore'), dict(color='C2', lw=1, label='F-Score $\emptyset$')),
                (('keys', 36, 'fscore'), dict(color='C3', lw=1, label='F-Score J')),
                (('keys', 57, 'fscore'), dict(color='C4', lw=1, label='F-Score Space')),
            ]

                    # (('keys', k, 'fscore'), dict(
                    #     color='C3' if k in (0, 35, 36, 57) else gray,
                    #     ls='solid' if k in (49, 35) else 'dashed',
                    #     lw=2 if k in (49, 35) else 1,
                    #     label="F-Score {}".format(
                    #         KEY_CODES[k] if k in (49, 35) else (
                    #             '$\emptyset$, J, Space' if k in (0, 35, 36, 57) else
                    #             'T, G, B, Y, U, M'
                    #         )
                    #     )
                    # ))

            # self.plots += sum([
            #     [
            #         # (('keys', k, 'precision'), dict(ls='dotted', color='C{}'.format((i+3) % 10))),
            #         # (('keys', k, 'recall'), dict(ls='dashed', color='C{}'.format((i+3) % 10))),
            #         (('keys', k, 'fscore'), dict(
            #             color='C3' if k in (0, 35, 36, 57) else gray,
            #             ls='solid' if k in (49, 35) else 'dashed',
            #             lw=2 if k in (49, 35) else 1,
            #             label="F-Score {}".format(
            #                 KEY_CODES[k] if k in (49, 35) else (
            #                     '$\emptyset$, J, Space' if k in (0, 35, 36, 57) else
            #                     'T, G, B, Y, U, M'
            #                 )
            #             )
            #         ))
            #     ]
            #     for i, k in enumerate([0] + self.config.key_codes)
            # ], [])

    @memoize
    def extract(self, i):
        training_cost = self.training_cost[i]
        actual = self.actual[i]
        predicted = self.predicted[i]
        testing_cost = self.testing_cost[i]

        matrix = confusion_matrix(actual, predicted, labels=None, sample_weight=None)

        # target_names = ['no key'] + [KEY_CODES[k] for k in self.config.key_codes]
        # print(classification_report(actual, predicted, target_names=target_names, digits=5))

        labels = list(range(len(self.config.key_codes) + 1))
        p, r, f1, s = precision_recall_fscore_support(actual, predicted, labels=labels, average=None)

        accuracy = accuracy_score(actual, predicted)

        keys = {
            k: dict(
                precision=p[i],
                recall=r[i],
                fscore=f1[i],
                support=s[i],
            )
            for i, k in enumerate([0,] + self.config.key_codes)
        }

        return dict(
            matrix=matrix,
            predicted=predicted,
            actual=actual,
            testing_cost=testing_cost,
            training_cost=training_cost,
            accuracy=accuracy,
            totals=dict(
                precision=np.average(p),
                recall=np.average(r),
                fscore=np.average(f1),
                support=np.sum(s),
            ),
            keys=keys,
        )

    def add(self, training_cost, actual, predicted, testing_cost):
        if self.test_size == 0:
            self.test_size = len(actual)
            self.training_cost = np.ndarray((0,), dtype=float)
            self.actual = np.ndarray((0,self.test_size), dtype=int)
            self.predicted = np.ndarray((0,self.test_size), dtype=int)
            self.testing_cost = np.ndarray((0,), dtype=float)

        self.count += 1
        self.training_cost = np.concatenate([self.training_cost, np.array([training_cost])])
        self.actual = np.concatenate([self.actual, np.array([actual])])
        self.predicted = np.concatenate([self.predicted, np.array([predicted])])
        self.testing_cost = np.concatenate([self.testing_cost, np.array([testing_cost])])

        self._update_live_plot()

    def print_row(self, epoch=None):
        if epoch == None:
            epoch = self.count - 1

        values = self.extract(epoch)

        keys_content = '\n'.join([
            '{name:>9} {precision:9.5f} {recall:9.5f} {fscore:9.5f} {support:9.0f}'.format(
                k=k, name=KEY_CODES[k], **values['keys'][k]
            )
            for k in [0] + self.config.key_codes
        ])

        print("""             Epoch {epoch}
--------------------------------------------------
Accuracy:       {accuracy:.5f}
Training cost:  {training_cost:.5f}
Testing cost:   {testing_cost:.5f}
--------------------------------------------------
      Key Precision    Recall   F-Score   Support
{keys_content}
==================================================""".format(epoch=epoch, keys_content=keys_content, **values))

    def load(self, filename):
        if filename.endswith('.json'):
            raw = ujson.load(open(filename, 'r'))
            print('JSON in memory')
            self.test_size = len(raw[0][1])
            self.count = len(raw)
            self.training_cost = np.ndarray(shape=(self.count,), dtype=float)
            self.actual = np.ndarray(shape=(self.count,self.test_size), dtype=int)
            self.predicted = np.ndarray(shape=(self.count,self.test_size), dtype=int)
            self.testing_cost = np.ndarray(shape=(self.count,), dtype=float)

            for i, row in enumerate(raw):
                self.testing_cost[i] = row[0]
                self.actual[i] = np.array(row[1])
                self.predicted[i] = np.array(row[2])
                self.training_cost[i] = row[3]

        else:
            with np.load(filename) as npz:
                self.training_cost = npz['training_cost']
                self.actual = npz['actual']
                self.predicted = npz['predicted']
                self.testing_cost = npz['testing_cost']

    def save(self, filename):
        if filename.endswith('.json'):
            raise Exception('Will only save stats as npz from now on.')
            # ujson.dump(self.raw, open(filename, 'w'))
        else:
            np.savez(filename,
                training_cost=self.training_cost,
                actual=self.actual,
                predicted=self.predicted,
                testing_cost=self.testing_cost)

    def draw(self, live_mode=False, filename=None, start=None, end=None, smooth=0, title=None):
        c = (max(self.count, end) if end else self.count) - (start or 0)

        if live_mode:
            plt.ion()

        plt.axis([1, max(25, c), 0, 1.1])
        xvalues = list(range(c))

        if title:
            plt.title(title)

        plt.xlabel('epoch')

        # plt.rc('font', size=40)
        # plt.rc('xtick', labelsize=16)
        # plt.rc('ytick', labelsize=16)
        # plt.rc('legend', fontsize=16)
        # plt.rc('axes', labelsize=16)

        self.plot_lines = []
        handles = {}

        for path, conf in self.plots:
            yvalues = [_get(self.extract(x), path) for x in xvalues]
            if smooth:
                yvalues = scipy.ndimage.filters.gaussian_filter1d(yvalues, sigma=smooth)

            line, = plt.plot(xvalues, yvalues, **conf)
            self.plot_lines.append(line)
            handles[conf['label']] = line

        plt.legend(handles=[x[1] for x in sorted(handles.items())])

        fig = plt.gcf()
        fig.tight_layout()

        if live_mode:
            plt.pause(0.01)
            plt.draw()
        elif filename:
            fig.set_size_inches(16, 8)
            plt.savefig(filename, dpi=120)
        else:
            plt.show()

    def start_live_plot(self):
        self.live_plot = True
        self.draw(True)

    def _update_live_plot(self):
        if not self.live_plot:
            return

        xvalues = list(range(self.count))
        for i, (path, conf) in enumerate(self.plots):
            yvalues = [_get(self.extract(x), path) for x in xvalues]
            self.plot_lines[i].set_xdata(xvalues)
            self.plot_lines[i].set_ydata(yvalues)

        plt.axis([0, max(25, self.count), 0, 1.1])
        plt.pause(0.01)
        plt.draw()
