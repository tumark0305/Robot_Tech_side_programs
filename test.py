import sympy as sp

t = sp.Symbol('t', real=True)
x0, y0 = 3, 2

# 假設軌跡
x_t = t
y_t = t**2

# 距離平方函數
D2 = (x_t - x0)**2 + (y_t - y0)**2

# 求導並找最小點
dD_dt = sp.diff(D2, t)
t_min_candidates = sp.solve(dD_dt, t)

# 驗證極小值
t_min_real = [tt.evalf() for tt in t_min_candidates if tt.is_real]

print("最短距離發生時間 t：", t_min_real)

# 計算最小距離平方值
min_dist_squared = min([D2.subs(t, tt).evalf() for tt in t_min_real])

# 真正距離（平方根）
min_distance = sp.sqrt(min_dist_squared).evalf()

print("最短距離長度：", min_distance)

