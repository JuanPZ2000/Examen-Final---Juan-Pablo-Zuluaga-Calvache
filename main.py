import numpy as np
import os
import cv2

# Examen final Juan Pablo Zuluaga Calvache


def click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))

def pixelPercentage(image):
    image_copy = image.copy()

    # Componente hue
    image_hsv = cv2.cvtColor(image_copy, cv2.COLOR_BGR2HSV)
    # Canal 0 ya que estamos analizando el hue
    hist = cv2.calcHist([image_hsv],[0], None, [180], [0, 180])

    max_val = hist.max()
    max_pos = int(hist.argmax())

    # Se halla la mascara para realizar el filtrado
    tolerance = 10
    inf_limit = (max_pos - tolerance, 0,0)
    sup_limit = (max_pos + tolerance, 255, 255)
    mask = cv2.inRange(image_hsv, inf_limit, sup_limit)

    # Como se tiene algunos huecos se puede hacer una erosion
    kernel = np.ones((5,5),np.uint8)
    mask_erosion = cv2.erode(mask, kernel, iterations= 1)

    zeros = np.count_nonzero(mask_erosion)
    porcentaje = zeros * 100 / (mask_erosion.shape[0]*mask_erosion.shape[1])
    print('el porcentaje de pixeles verdes de la imagen es: '+str(porcentaje)+"%");

    cv2.imshow('primer punto',mask_erosion)
    cv2.waitKey(0)

    return mask_erosion


def segundoPunto(mask, image):
    # Se hace una dilatacion para quitar las lineas del campo
    conta = 0
    kernel = np.ones((16,16), np.uint8)
    mask_dilation = cv2.dilate(mask, kernel, iterations=1)

    # Se vuelve a erosionar para que se noten mejor los contornos
    kernel = np.ones((16,16), np.uint8)
    mask_erotion = cv2.erode(mask_dilation, kernel, iterations= 1)

    # Se encuentran los contornos para contar los jugadores
    contours, hierarchy = cv2.findContours(mask_erotion, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    image_draw = image.copy()
    for idx, cont in enumerate(contours):
        # Se detecta un contorno que este entre el valor superior e inferior, para evitar contornos muy grandes o muy pequenos
        if len(contours[idx]) < 350 and len(contours[idx]) > 21:
            x, y, width, height = cv2.boundingRect(contours[idx])
            cv2.rectangle(image_draw, (x, y), (x + width, y + height), (0, 0, 255), 2)
            # Se suma uno al contador de jugadores
            conta += 1

    cv2.imshow('segundo punto', image_draw)
    cv2.waitKey(0)
    print('se detectaron '+str(conta)+' jugadores')

def tercerPunto(image,points):
    image_draw = np.copy(image)
    points1 = []
    points2 = []

    cv2.namedWindow("Image")
    cv2.setMouseCallback("Image", click)
    # While para los primeros dos puntos p1 y p2
    point_counter = 0
    while True:
        cv2.imshow("Image", image_draw)
        key = cv2.waitKey(1) & 0xFF

        if point_counter > 2:
            break;
        if len(points) > point_counter:
            point_counter = len(points)
            cv2.circle(image_draw, (points[-1][0], points[-1][1]), 10, [255, 0, 0],-1)

    point_counter = 0
    # While para el tercer punto P3
    while True:
        cv2.imshow("Image", image_draw)
        key = cv2.waitKey(1) & 0xFF
        if point_counter > 1:
            break;
        if len(points) > point_counter:
            point_counter = len(points)
            cv2.circle(image_draw, (points[-1][0], points[-1][1]), 10, [255, 0, 255], -1)
    points1.append(points[0])
    points1.append(points[1])
    points2.append(points[2])
    cv2.destroyAllWindows()

    # Se encuentra la pendiente teniendo los puntos x,y
    m1 = (points1[1][1]-points1[0][1]) / (points1[1][0]-points1[0][0])
    b1 = points1[1][1] - (m1 * points1[1][0])

    # Se pone un x al inicio y al final de la imagen
    x1 = 0
    x2 = image.shape[1]

    # Se encuentra el y para cada x
    y1 = int(m1*x1 + b1)
    y2 = int(m1*x2 + b1)

    cv2.line(image_draw, (x1, y1), (x2, y2), (255, 0, 0), thickness=3, lineType=8)

    # Segunda recta
    # Nuevo punto de corte
    b1 = points2[0][1] - (m1 * points2[0][0])

    # Se pone un x al inicio y al final de la imagen
    x1 = 0
    x2 = image.shape[1]

    # Se encuentra el y para cada x
    y1 = int(m1 * x1 + b1)
    y2 = int(m1 * x2 + b1)
    cv2.line(image_draw, (x1, y1), (x2, y2), (255, 0, 255), thickness=3, lineType=8)

    cv2.imshow('tercer punto',image_draw)
    cv2.waitKey(0)


if __name__ == '__main__':
    points = []
    path = os.path.abspath(os.getcwd()) + '\soccer_game.png'
    image = cv2.imread(path)
    mask = pixelPercentage(image)
    segundoPunto(mask,image)
    tercerPunto(image,points)
