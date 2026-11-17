
from tdmclient import ClientAsync

# Parametres du robot
SEUIL_OBSTACLE = 2000
VITESSE_AVANCE = 300
VITESSE_ROTATION = 100

with ClientAsync() as client:

    async def programme():

        with await client.lock() as node:

            # Attendre les variables du robot
            await node.wait_for_variables({
                "prox.horizontal",
                "motor.left.target",
                "motor.right.target"
            })

            # Arreter le robot au demarrage
            node.v.motor.left.target = 0
            node.v.motor.right.target = 0
            node.flush()

            await client.sleep(1.0)

            etat = "AVANCE"
            direction = None

            try:
                while True:

                    # Lire les capteurs
                    capteurs = list(node.v.prox.horizontal)

                    gauche = capteurs[1]
                    centre = capteurs[2]
                    droite = capteurs[3]

                    # Detecter les obstacles
                    obstaclegauche = gauche > SEUIL_OBSTACLE
                    obstaclecentre = centre > SEUIL_OBSTACLE
                    obstacledroite = droite > SEUIL_OBSTACLE

                    obstacle = (
                        obstaclegauche
                        or obstaclecentre
                        or obstacledroite
                    )

                    if etat == "AVANCE":

                        if obstacle:

                            etat = "TOURNE"

                            if obstaclecentre:

                                if gauche < droite:
                                    direction = "GAUCHE"
                                else:
                                    direction = "DROITE"

                            elif obstaclegauche:
                                direction = "DROITE"

                            elif obstacledroite:
                                direction = "GAUCHE"

                            print(
                                "Obstacle detecte !",
                                "Rotation :", direction
                            )

                    elif etat == "TOURNE":

                        if not obstacle:
                            etat = "AVANCE"
                            direction = None
                            print("Chemin libre !")

                    # Choisir une seule commande moteur
                    if etat == "AVANCE":

                        node.v.motor.left.target = VITESSE_AVANCE
                        node.v.motor.right.target = VITESSE_AVANCE

                    elif direction == "DROITE":

                        node.v.motor.left.target = VITESSE_ROTATION
                        node.v.motor.right.target = -VITESSE_ROTATION

                    elif direction == "GAUCHE":

                        node.v.motor.left.target = -VITESSE_ROTATION
                        node.v.motor.right.target = VITESSE_ROTATION

                    # Envoyer les commandes
                    node.flush()

                    await client.sleep(0.1)

            finally:

                # Arreter le robot en quittant
                node.v.motor.left.target = 0
                node.v.motor.right.target = 0
                node.flush()

                print("Robot arrete.")

    client.run_async_program(programme)
