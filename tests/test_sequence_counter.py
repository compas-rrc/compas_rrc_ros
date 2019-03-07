import pytest
from abb_042_driver.message import SequenceCounter


def test_thread_safety():
    import threading

    counter = SequenceCounter()
    threads = []

    def incrementor():
        for _ in range(100000):
            counter.increment()

    for _ in range(4):
        thread = threading.Thread(target=incrementor)
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()

    assert counter.value == 400000
