# Import der erforderlichen Bibliotheken
import rclpy  # ROS2 Python-Bibliothek
from rclpy.node import Node  # Basisklasse für Nodes in ROS2
from sensor_msgs.msg import Image  # Nachrichtentyp für Bildnachrichten in ROS2
import cv2  # OpenCV-Bibliothek für Bildverarbeitung
from cv_bridge import CvBridge  # Hilfsklasse zum Konvertieren zwischen ROS- und OpenCV-Bildformaten
import numpy as np  # Bibliothek für numerische Operationen

# Definition der Klasse LineDetector, die von Node erbt
class LineDetector(Node):
    def __init__(self):
        # Konstruktor der Basisklasse initialisieren
        super().__init__('line_detector')
        # Erstellung eines Subscribers, der auf Bildnachrichten vom Kamera-Publisher hört
        self.subscription = self.create_subscription(
            Image,
            '/camera_image',
            self.image_callback,
            10)  # QoS-Profil mit einer Tiefe von 10
        # Instanz von CvBridge, um zwischen ROS- und OpenCV-Bildern zu konvertieren
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Konvertiere ROS Image Nachricht zu einem OpenCV-Bild
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Filtere alle Farben außer Weiß
        lower_white = np.array([200, 200, 200], dtype="uint8")
        upper_white = np.array([255, 255, 255], dtype="uint8")
        white_mask = cv2.inRange(cv_image, lower_white, upper_white)
        white_image = cv2.bitwise_and(cv_image, cv_image, mask=white_mask)

        # Finden der Konturen der weißen Bereiche
        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centers = []

        # Berechne die Zentren der weißen Konturen
        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])  # X-Koordinate des Schwerpunkts
                cy = int(M["m01"] / M["m00"])  # Y-Koordinate des Schwerpunkts
                centers.append((cx, cy))

        # Berechne den Mittelpunkt zwischen den weißen Linien und zeichne den Punkt
        if len(centers) >= 2:
            midpoint = sum([pt[0] for pt in centers]) / len(centers)
            height, width = white_image.shape[:2]
            center_screen = (width // 2, height)
            cv2.circle(white_image, center_screen, 10, (255, 0, 0), -1)
            cv2.circle(white_image, (int(midpoint), height // 2), 10, (0, 255, 0), -1)
            cv2.line(white_image, center_screen, (int(midpoint), height // 2), (0, 0, 255), 2)

            # Logge die X-Koordinate des Mittelpunkts
            self.get_logger().info(f'Midpoint X Coordinate: {midpoint}')

        # Zeige das gefilterte Bild an
        cv2.imshow('Filtered Image', white_image)
        cv2.waitKey(1)  # Wichtig, um das Bild anzuzeigen und UI-Ereignisse zu verarbeiten

# Hauptfunktion, die beim Ausführen des Scripts aufgerufen wird
def main(args=None):
    rclpy.init(args=args)  # Initialisiere ROS2
    line_detector = LineDetector()  # Erstelle eine Instanz der LineDetector-Klasse
    rclpy.spin(line_detector)  # Führe die Node aus
    line_detector.destroy_node()  # Zerstöre die Node, wenn fertig
    rclpy.shutdown()  # Schalte ROS2 herunter

if __name__ == '__main__':
    main()  # Führe die Hauptfunktion aus, wenn das Script direkt ausgeführt wird

