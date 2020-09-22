import math
import plotly
import json
from flask import Flask, render_template, request
import mpc_sim
import numpy as np

app = Flask(__name__)
global_Tp = 0.1


def run_simulation(sim_parameters):
    sim_results = []
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
    e = zadany_poziom - poziom
    e_poprzedni = 0
    ListaPoziom = []
    ListaN = []
    ListaU = []
    ListaE = []

    # charakterystyka zaworu 0- zamknięty 10-otwarty na max
    u_max = 10
    u_min = 0

    u = kp * (e + Tp / Ti * e_sum + Td / Tp * (e - e_poprzedni))
    if u > u_max:
        u = u_max
    if u < u_min:
        u = u_min

    stop_condition = False
    stop_counter = 0

    while t < czas_sym:
        ListaN.append(t)
        ListaPoziom.append(poziom)
        ListaE.append(e)
        ListaU.append(u)

        u = kp * (e + Tp / Ti * e_sum + Td / Tp * (e - e_poprzedni))
        if u > u_max:
            u = u_max
        if u < u_min:
            u = u_min

        # Tp = global_Tp / (1 + 0.01 * abs(u - u_poprzedni) / Tp)
        poziom = (1 / A) * (-B * math.sqrt(poziom) + u) * Tp + poziom
        if poziom < 0: poziom = 0
        if poziom > 10: poziom = 10
        e = zadany_poziom - poziom
        t += Tp
        if u < u_max and u > 0:
            e_sum += e
        e_poprzedni = e
        if abs(e) < 0.01 * abs(zadany_poziom - pocz_poziom):
            stop_counter += 1
        if stop_counter > 3 and not stop_condition:
            czas_sym = 1.5 * t
            stop_condition = True

    sim_results.append(ListaN)
    sim_results.append(ListaPoziom)
    sim_results.append(ListaU)
    sim_results.append(ListaE)

    return sim_results, t


@app.route('/', methods=['POST', 'GET'])
def index():
    if request.method == 'POST':
        result = request.form

        # Ziegler-Nichols
        Ku = 100
        Pu = 0.2

        zn_kp = Ku * 0.6
        zn_Ti = Pu / 2.0
        zn_Td = Pu / 8.0

        initial_lvl = float(result["initial_lvl"])
        desired_lvl = float(result["desired_lvl"])
        A = float(result["A"])
        beta = float(result["beta"])
        Tp = float(result["Tp"])

        if initial_lvl == 0:
            initial_lvl = 0.001

        user_parameters = {
            "czas_sym": 30,
            "kp": zn_kp,
            "Tp": Tp,
            # "Td": 0,
            # "Ti": float('inf'),
            "Td": zn_Td / 10,
            "Ti": zn_Ti * 10,
            "A": A,
            "B": beta,
            "pocz_poz": initial_lvl,
            "zad_poz": desired_lvl
        }

        pid_results, sim_time = run_simulation(user_parameters)

        user_parameters['czas_sym'] = sim_time
        mpc_results = mpc_sim.run_mpc_simulation(user_parameters)

        ids, plot_json = create_plot(pid_results, mpc_results, desired_lvl)

        return render_template('index.html', ids=ids, plot=plot_json, sim_params=user_parameters, y=desired_lvl*10)

    return render_template('index.html')


def create_plot(pid, mpc, desired_lvl):
    lista_t = pid[0]
    ListaPoziom = pid[1]
    ListaU = pid[2]
    ListaE = pid[3]

    h_mpc = list(mpc['h'].flatten())
    u_mpc = list(mpc['u'].flatten())
    e_mpc = []
    for h in h_mpc:
        e = float(desired_lvl) - h
        e_mpc.append(e)

    graphs = [
        dict(
            data=[
                dict(
                    x=lista_t,
                    y=ListaPoziom,
                    type='scatter',
                    name='PID',
                    line=dict(
                        shape='spline'
                    )
                ),
                dict(
                    x=lista_t,
                    y=h_mpc,
                    type='scatter',
                    name='MPC',
                    line=dict(
                        shape='spline'
                    )
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
                    type='scatter',
                    name='PID'
                ),
                dict(
                    x=lista_t,
                    y=u_mpc,
                    type='scatter',
                    name='MPC',
                    line=dict(
                        shape='hv'
                    )
                )
            ],
            layout=dict(
                title='<b>Zadane natężenie dopływu</b>',
                xaxis=dict(
                    title='t [s]'
                ),
                yaxis=dict(
                    title='Natężenie dopływu [m<sup>3</sup>/s]'
                )

            )
        ),

        dict(
            data=[
                dict(
                    x=lista_t,
                    y=ListaE,
                    type='scatter',
                    name='PID',
                    line=dict(
                        shape='spline'
                    )
                ),
                dict(
                    x=lista_t,
                    y=e_mpc,
                    type='scatter',
                    name='MPC',
                    line=dict(
                        shape='spline'
                    )
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
