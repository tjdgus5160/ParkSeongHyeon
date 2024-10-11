import numpy as np
import matplotlib.pyplot as plt

# 에너지 값을 생성합니다.
energies = np.linspace(-10, 10, 100)

# 볼츠만 분포를 계산합니다.
probabilities = np.exp(-energies / (1.38e-23 * 298))

# 그래프를 그립니다.
plt.plot(energies, probabilities)
plt.xlabel("Energy (J)")
plt.ylabel("Probability")
plt.show()