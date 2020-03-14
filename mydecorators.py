import time

def measure_time(func):
    """measures the time that a calculation took"""
    def wrapper(*args, **kwargs):
        start = time.time()
        calculation = func(*args, **kwargs)
        end = time.time()
        print('\nFunction: %s\nCalculation took %.2f sek\n' % (
            func.__name__,
            end - start))
        return calculation

    return wrapper