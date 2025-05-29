import shutil
import os
import random
import re
import argparse

def modificar_valor_linea(linea, nuevo_valor):
    indent = re.match(r"^\s*", linea).group(0)
    tiene_coma = linea.rstrip().endswith(",")
    coma = "," if tiene_coma else ""
    nueva_linea = f"{indent}{nuevo_valor:.7f}{coma}\n"
    return nueva_linea

def modificar_linea(linea, rango):
    nuevo_valor = random.uniform(*rango)
    return modificar_valor_linea(linea, nuevo_valor)

def intercambiar_posiciones_pares(lineas, lineas_x, lineas_y):
    # Extraer pares (x, y)
    pares = []
    for idx_x, idx_y in zip(lineas_x, lineas_y):
        x_line = lineas[idx_x - 1]
        y_line = lineas[idx_y - 1]
        x_val = float(re.search(r"([-+]?\d*\.\d+|\d+)", x_line).group(0))
        y_val = float(re.search(r"([-+]?\d*\.\d+|\d+)", y_line).group(0))
        pares.append((x_val, y_val))

    # Barajar los pares completos
    random.shuffle(pares)

    # Reasignar pares mezclados a las líneas
    for i, (idx_x, idx_y) in enumerate(zip(lineas_x, lineas_y)):
        x_nuevo, y_nuevo = pares[i]
        lineas[idx_x - 1] = modificar_valor_linea(lineas[idx_x - 1], x_nuevo)
        lineas[idx_y - 1] = modificar_valor_linea(lineas[idx_y - 1], y_nuevo)

    return lineas

def copiar_y_modificar_prefab(origen, carpeta_destino, lineas_x, lineas_y, rangos, lineas_x_toy, lineas_y_toy):
    if os.path.exists(carpeta_destino):
        print(f"Eliminando todo el contenido en: {carpeta_destino}")
        for nombre in os.listdir(carpeta_destino):
            ruta = os.path.join(carpeta_destino, nombre)
            if os.path.isfile(ruta) or os.path.islink(ruta):
                os.unlink(ruta)
            elif os.path.isdir(ruta):
                shutil.rmtree(ruta)
    else:
        os.makedirs(carpeta_destino)

    nombre_archivo = "RandomPositions.prefab"
    destino = os.path.join(carpeta_destino, nombre_archivo)

    shutil.copy2(origen, destino)
    print(f"Archivo copiado a: {destino}")

    with open(destino, 'r', encoding='utf-8') as f:
        lineas = f.readlines()

    # 1. Intercambiar posiciones de objetos TOY
    if lineas_x_toy and lineas_y_toy:
        lineas = intercambiar_posiciones_pares(lineas, lineas_x_toy, lineas_y_toy)
        print("Posiciones intercambiadas entre objetos pequeños del objeto TOY.")

    # 2. Modificar posiciones aleatorias de cubos
    for n in lineas_x:
        idx = n - 1
        if 0 <= idx < len(lineas):
            lineas[idx] = modificar_linea(lineas[idx], rangos['x'])
            print(f"Línea {n} (x) modificada con posición aleatoria.")
        else:
            print(f"Advertencia: línea {n} (x) fuera del rango.")

    for n in lineas_y:
        idx = n - 1
        if 0 <= idx < len(lineas):
            lineas[idx] = modificar_linea(lineas[idx], rangos['y'])
            print(f"Línea {n} (y) modificada con posición aleatoria.")
        else:
            print(f"Advertencia: línea {n} (y) fuera del rango.")

    with open(destino, 'w', encoding='utf-8') as f:
        f.writelines(lineas)

    print("Archivo modificado y guardado.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Copiar un archivo .prefab y modificar posiciones X y Y.")
    parser.add_argument("--origen", required=True, help="Ruta completa del archivo .prefab original")
    parser.add_argument("--destino", required=True, help="Carpeta destino donde se copiará el archivo")
    args = parser.parse_args()

    # Líneas de cubos a modificar aleatoriamente
    lineas_x = [2223, 2828, 3433, 4038, 4643, 5248]
    lineas_y = [2224, 2829, 3434, 4039, 4644, 5249]

    # Líneas de objetos pequeños dentro de TOY (a intercambiar)
    lineas_x_toy = [1257, 1375, 1919]  
    lineas_y_toy = [1258, 1376, 1920]  

    rangos = {
        'x': (1.456009, 2.2069075),
        'y': (1.9625132, 3.4060755)
    }

    copiar_y_modificar_prefab(args.origen, args.destino, lineas_x, lineas_y, rangos, lineas_x_toy, lineas_y_toy)
