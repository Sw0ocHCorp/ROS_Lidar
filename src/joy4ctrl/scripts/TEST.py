def convertir_interval_angle(angle):
    if (abs(angle)):
        angle%=360
    elif angle < 0:
        angle += 360
    return angle

# Exemple d'utilisation :
angle_degre = 710  # Un angle dans l'intervalle [-180, 180]
angle_converti = convertir_interval_angle(angle_degre)
print("Angle converti en degrés:", angle_converti) 