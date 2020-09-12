import math
import plotly
import json
from flask import Flask, render_template, request

app = Flask(__name__)
sim_results = []
global_Tp = 0.05


def runSimulation(sim_parameters):
    czas_sym = sim_parameters["czas_sym"]
    kp = sim_parameters["kp"]
    Tp = sim_parameters["Tp"]
    Td = sim_parameters["Td"]
    Ti = sim_parameters["Ti"]
    A = sim_parameters["A"]
    B = sim_parameters["B"]
    pocz_poziom = float(sim_parameters["pocz_poz"])
    zadany_poziom = float(sim_parameters["zad_poz"])

    poziom = pocz_poziom
    e_sum = 0
    t = 0
    e_poprzedni = 0
    u_poprzedni = 0
    ListaPoziom = []
    ListaN = []
    ListaU = []
    ListaE = []

    # charakterystyka zaworu 0- zamknięty 10-otwarty na max
    u_max = 10
    u_min = 0

    stop_condition = False
    stop_counter = 0

    while t < czas_sym:
        ListaN.append(t)
        ListaPoziom.append(poziom)
        e = zadany_poziom - poziom
        ListaE.append(e)
        u = kp * (e + Tp / Ti * e_sum + Td / Tp * (e - e_poprzedni))
        if u > u_max:
            u = u_max
        if u < u_min:
            u = u_min
        ListaU.append(u)
        Tp = global_Tp / (1 + 0.01 * abs(u - u_poprzedni) / Tp)
        poziom = (1 / A) * (-B * math.sqrt(poziom) + u) * Tp + poziom
        if poziom < 0: poziom = 0
        if poziom > 100: poziom = 100
        t += Tp
        if u < u_max and u > 0:
            e_sum += e
        e_poprzedni = e
        u_poprzedni = u
        if abs(e) < 0.01 * abs(zadany_poziom - pocz_poziom):
            stop_counter += 1
        if stop_counter > 3 and not stop_condition:
            czas_sym = t + 1 * t
            stop_condition = True

    sim_results.clear()
    sim_results.append(ListaN)
    sim_results.append(ListaPoziom)
    sim_results.append(ListaU)
    sim_results.append(ListaE)


@app.route('/', methods=['POST', 'GET'])
def index():
    if request.method == 'POST':
        result = request.form

        # Ziegler-Nichols
        Ku = 202
        Pu = 0.1

        zn_kp = Ku * 0.6
        zn_Ti = Pu / 2.0
        zn_Td = Pu / 8.0

        user_parameters = {
            "czas_sym": 500,
            "kp": zn_kp,
            "Tp": global_Tp,
            "Td": 0,
            # "Ti": float('inf'),
            "Td": zn_Td,
            "Ti": zn_Ti*10,
            "A": 5,
            "B": 0.5,
            "pocz_poz": result["initial_lvl"],
            "zad_poz": result["desired_lvl"]
        }

        runSimulation(user_parameters)
        ids, plot_json = create_plot()

        return render_template('index.html', ids=ids, plot=plot_json, sim_params=user_parameters)

    return render_template('index.html')


def create_plot():
    lista_t = sim_results[0]
    ListaPoziom = sim_results[1]
    ListaU = sim_results[2]
    ListaE = sim_results[3]

    graphs = [
        dict(
            data=[
                dict(
                    x=lista_t,
                    y=ListaPoziom,
                    type='scatter'
                )
            ],
            layout=dict(
                title='<b>Poziom substancji w zbiorniku</b>',
                xaxis=dict(
                    title=dict(text='t [s]')
                ),
                yaxis=dict(
                    title='Poziom [m]'
                )
            )
        ),

        dict(
            data=[
                dict(
                    x=lista_t,
                    y=ListaU,
                    type='scatter'
                )
            ],
            layout=dict(
                title='<b>Sygnał sterujący</b>',
                xaxis=dict(
                    title='t [s]'
                ),
                yaxis=dict(
                    title='Wartość sygnału sterującego'
                )

            )
        ),

        dict(
            data=[
                dict(
                    x=lista_t,
                    y=ListaE,
                    type='scatter'
                )
            ],
            layout=dict(
                title='<b>Uchyb regulacji</b>',
                xaxis=dict(
                    title='t [s]'
                ),
                yaxis=dict(
                    title='Wartość uchybu [m]'
                )

            )
        )
    ]

    ids = ['graph-{}'.format(i) for i, _ in enumerate(graphs)]
    graphJSON = json.dumps(graphs, cls=plotly.utils.PlotlyJSONEncoder)

    return ids, graphJSON
