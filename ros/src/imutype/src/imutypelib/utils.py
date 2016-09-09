def coroutine(func):
    def start(*args, **kwargs):
        cr = func(*args, **kwargs)
        next(cr)
        return cr
    return start

class Collector(object):
    def __init__(self, max_count=None):
        self.items = []
        self.max_count = max_count

    @coroutine
    def __call__(self, target=None):
        while True:
            value = (yield)
            self.items.append(value)
            if self.max_count != None:
                self.items = self.items[-self.max_count:]
            if target:
                target.send(self.items)

