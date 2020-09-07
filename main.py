import math
import matplotlib.pyplot as plt


def powitanie():
    name = input("Podaj swoje imie: ")
    print("Witaj {name}".format(name=name))


CzasProb = 0.1
CzasSym = 1000
kp = 47
Tp = 0.01
Td = 0
Ti = float('inf')
A = 4
B = 0.25
PoczPoziom = float(input("Podaj poczatkowy poziom cieczy w zbiorniku [m]: "))
ZadanyPoziom = float(input("Podaj zadany poziom cieczy, MIN 0, MAX 100:"))
print("Symulacja rozpoczyna się")

# powitanie()  # Powitanie, bo czemu by nie

n = CzasSym/CzasProb   # Liczba próbek
print("Liczba próbek {n}".format(n=n))

Poziom = PoczPoziom
eSum = 0
ePoprzedni = 0
ListaPoziom = []
ListaN = []
ListaU = []
for i in range(int(n)):
    e = ZadanyPoziom-Poziom
    u = kp * (e+Tp/Ti * eSum + Td / Tp * (e-ePoprzedni))
# charakterystyka zaworu 0- zamknięty 10-otwarty na max
    if u > 10:
        u = 10
    if u < 0:
        u = 0
    ListaU.append(u)
    Poziom = (1/A)*(-B*math.sqrt(Poziom)+u)*CzasProb+Poziom
    ListaPoziom.append(Poziom)
    ListaN.append(i)
    eSum = eSum+e
    ePoprzedni = e


plt.subplot(2, 1, 1)
plt.plot(ListaN, ListaPoziom, 'r')
plt.grid(True)
plt.subplot(2, 1, 2)
plt.plot(ListaN, ListaU, 'g')
plt.grid(True)
plt.show()
