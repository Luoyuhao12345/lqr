from math import sqrt, pow


def find_closet_point(x, y, path_x, path_y):
    N = len(path_x)
    min_dis = 99999
    index = 0
    for i in range(N):
        d_x = x- path_x[i]
        d_y = y - path_y[i]
        cur_dis = sqrt(pow(d_x, 2) + pow(d_y, 2))
        if cur_dis < min_dis:
            min_dis = cur_dis
            index = i
    return index


def find_forward_point(x, y, path_x, path_y):
    lf = 0.1
    index = find_closet_point(x, y, path_x, path_y)
    f_index = index
    N1 = len(path_x)
    for i in range(index, N1):
        d_x = path_x[index] - path_x[i]
        d_y = path_y[index] - path_y[i]
        cur_dis = sqrt(pow(d_x, 2) + pow(d_y, 2))
        f_index = i
        if cur_dis >= lf:
            break
    print("lf:{:.2f}".format(lf))
    return f_index


# 将自定义的局部路径与原来写的函数兼容
def ref_path2xy(path):
    n = len(path)
    xs = list(range(n))
    ys = list(range(n))
    for i in range(n):
        xs[i] = path[i].x
        ys[i] = path[i].y
    return xs, ys

