
import tkinter as tk
import threading
import queue

from tdmclient import ClientAsync

SEUIL = 2000

# File de communication entre le Thymio et la fenetre graphique
file_mesures = queue.Queue()

# Demande d'arret
arret = threading.Event()

# Lecture des capteurs
def lire_capteurs():

    try:
        with ClientAsync() as client:

            async def programme():

                with await client.lock() as node:

                    print("Thymio connecte !")

                    await node.wait_for_variables({"prox.horizontal"})
                    
                    # Actualisation des valeurs initiales
                    await client.sleep(1.0)

                    while not arret.is_set():

                        valeurs = list(node.v.prox.horizontal)

                        # Les 5 capteurs avant
                        file_mesures.put(valeurs[:5])

                        await client.sleep(0.1)

            client.run_async_program(programme)

    except Exception as erreur:
        print("Erreur :", erreur)

    finally:
        arret.set()


# Interface graphique
fenetre = tk.Tk()
fenetre.title("Capteurs infrarouges - Thymio")
fenetre.geometry("550x430")
fenetre.resizable(False, False)

canvas = tk.Canvas(
    fenetre,
    width=550,
    height=330,
    bg="white"
)
canvas.pack()

# 2 cases sur les cotes
# 3 cases devant
positions = [
    (90, 165),   # Capteur 0 : gauche
    (170, 75),   # Capteur 1 : avant gauche
    (245, 75),   # Capteur 2 : avant centre
    (320, 75),   # Capteur 3 : avant droit
    (395, 165)   # Capteur 4 : droite
]

cases = []

for i, (x, y) in enumerate(positions):

    rectangle = canvas.create_rectangle(
        x,
        y,
        x + 60,
        y + 60,
        fill="white",
        outline="gray",
        width=2
    )

    cases.append(rectangle)

    canvas.create_text(
        x + 30,
        y + 30,
        text=str(i),
        fill="gray",
        font=("Arial", 12, "bold"),
        tags=f"texte_{i}"
    )


# Graphisme robot
canvas.create_rectangle(
    205,
    205,
    345,
    305,
    fill="#3b82f6",
    outline="#1d4ed8",
    width=3
)

canvas.create_text(
    275,
    265,
    text="THYMIO",
    fill="white",
    font=("Arial", 15, "bold")
)

canvas.create_polygon(
    275, 215,
    262, 235,
    288, 235,
    fill="white"
)



# Affichage des valeurs
label_valeurs = tk.Label(
    fenetre,
    text="En attente des capteurs...",
    font=("Arial", 11)
)
label_valeurs.pack(pady=8)



# Actualisation des valeurs
def actualiser():

    derniere_mesure = None

    # Recuperer la mesure la plus recente
    while not file_mesures.empty():
        derniere_mesure = file_mesures.get()

    if derniere_mesure is not None:

        valeurs = derniere_mesure

        for i in range(5):

            if valeurs[i] > SEUIL:
                couleur = "black"
                couleur_texte = "white"
            else:
                couleur = "white"
                couleur_texte = "gray"

            canvas.itemconfig(
                cases[i],
                fill=couleur
            )

            canvas.itemconfig(
                f"texte_{i}",
                fill=couleur_texte
            )

        label_valeurs.config(
            text=" | ".join(
                f"C{i}: {valeurs[i]}"
                for i in range(5)
            )
        )

    if not arret.is_set():
        fenetre.after(100, actualiser)



# Fermeture
def fermer():

    arret.set()
    fenetre.destroy()


fenetre.protocol(
    "WM_DELETE_WINDOW",
    fermer
)

# Demarrage
thread = threading.Thread(
    target=lire_capteurs,
    daemon=True
)

thread.start()

fenetre.after(100, actualiser)
fenetre.mainloop()
