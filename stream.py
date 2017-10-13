class Stream:
    def frames(self):
        raise NotImplementedError

    def __enter__(self):
        raise NotImplementedError

    def __exit__(self, exc_type, exc_val, exc_tb):
        raise NotImplementedError

    def start(self):
        self.__enter__()

    def stop(self):
        self.__exit__()