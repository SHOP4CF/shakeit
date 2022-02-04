from math import floor, sin, pi, cos
import matplotlib.pyplot as plt
from time import time


def asymmetric_triangle_wave(t, tmax, m):
    t = t - (2 * tmax * floor(t / (2 * tmax)))
    if t <= tmax / m:
        return (m * t) / tmax
    elif tmax / m <= t <= (2 * tmax) - (tmax / m):
        return 1 - ((m / ((m - 1) * tmax)) * (t - (tmax / m)))
    else:
        return (m / tmax) * (t - (2 * tmax))


def fourier_triangle_wave(t, tmax, m, n, phase=0):
    phase = (phase * pi)/180
    bn = lambda n, m: ((2 * m**2 * (-1)**n)/(n**2 * (m - 1) * pi**2)) * sin((n * pi * (m - 1))/m)
    g = lambda n, t, tmax, bn: bn * sin(phase + (n * pi * t)/tmax)
    return sum([g(i, t, tmax, bn(i, m)) for i in range(1, n)])


def plot(ax, xs, m, n):
    t0 = time()
    ys = [fourier_triangle_wave(x, max(xs)/2, m, n) for x in xs]
    ax.plot(xs, ys, label=f'm={m}, n={n}, t={(time() - t0) * 100000:.0f} us', linewidth=0.75)


def plot_1d_velocity():
    period = 1
    ts = [0.001 * x for x in range(1001)]

    fig, ax = plt.subplots()

    for m, n in zip([5, 10, 30], [10, 27, 1000]):
        ax.plot(ts, [-asymmetric_triangle_wave(t, period/2, m=m) for t in ts], '--', label=f'unsym, m={m}', alpha=0.7)
        plot(ax, ts, m=m, n=n)

    ax.set_title('Fourier series approximation\nof asymmetric triangle wave')
    fig.legend(frameon=False, loc='lower center', ncol=6)
    fig.show()


def position_1d(ts, period, m, phase=0):
    dt = ts[1] - ts[0]
    vs = [-fourier_triangle_wave(t, period/2, m=m, n=60, phase=phase) for t in [dt * x for x in range(1001)]]

    ps = [0.0]
    for v in vs: ps.append(ps[-1] + (v * dt))

    return ps


def plot_1d_position():
    period = 1
    dt = 0.001
    ts = [dt * x for x in range(1001)]
    ms = [200, 200]

    fig, ax = plt.subplots()

    for m, phase in zip(ms, [0, 90]):
        ax.scatter(ts, position_1d(ts, period, m=m, phase=phase)[:-1], label=f'm={m}, ph={phase}°', s=0.75)

    ax.set_title('1D position in velocity-based control with different ph shifts')
    fig.legend(frameon=False, loc='lower center', ncol=len(ms))
    fig.show()


def plot_2d_position():
    period = 1
    dt = 0.001
    ts = [dt * x for x in range(1001)]

    fig, ax = plt.subplots()

    pxs = position_1d(ts, period, m=200, phase=0)[:-1]
    pys = position_1d(ts, period/2, m=200, phase=90)[:-1]

    ax.scatter(pxs, pys, s=0.75)

    ax.set_title('2D position')
    fig.legend(frameon=False, loc='lower center')
    fig.show()


if __name__ == '__main__':
    plot_1d_velocity()
    plot_1d_position()
    plot_2d_position()
    q = input('press any key to exit')
