import numpy as np
import matplotlib.pyplot as plt

# ������ ���� �����մϴ�.
energies = np.linspace(-10, 10, 100)

# ������ ������ ����մϴ�.
probabilities = np.exp(-energies / (1.38e-23 * 298))

# �׷����� �׸��ϴ�.
plt.plot(energies, probabilities)
plt.xlabel("Energy (J)")
plt.ylabel("Probability")
plt.show()