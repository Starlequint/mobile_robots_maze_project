from tdmclient import ClientAsync

def lire_capteurs():

    with ClientAsync() as client:

        async def programme():

            with await client.lock() as node:

                await node.wait_for_variables({"prox.horizontal"})

                # Attendre les premières actualisations
                await client.sleep(1.0)

                while True:

                    # Lire les 5 capteurs avant
                    valeurs = list(node.v.prox.horizontal[:5])

                    # Afficher les valeurs en temps réel
                    print(valeurs)

                    # Attendre 100 ms
                    await client.sleep(0.1)

        client.run_async_program(programme)


lire_capteurs()