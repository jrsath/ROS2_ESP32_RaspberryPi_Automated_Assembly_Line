# Remember a closed window so the next frame cannot recreate it.
import cv2


class CameraWindow:
    def __init__(self, title="Project 2 - Camera inspection"):
        self.title = title
        self.created = False

    def visible(self):
        if not self.created:
            return True
        try:
            visible = cv2.getWindowProperty(self.title, cv2.WND_PROP_VISIBLE)
            if visible >= 0:
                return visible >= 1
            # Ubuntu's GTK backend does not implement WND_PROP_VISIBLE (-1).
            # AUTOSIZE still distinguishes an existing window from a closed one.
            return cv2.getWindowProperty(self.title, cv2.WND_PROP_AUTOSIZE) >= 0
        except cv2.error:
            return False

    def draw(self, image):
        # Check before imshow: imshow would otherwise reopen an X-closed window.
        if not self.visible():
            return False
        if not self.created:
            cv2.namedWindow(self.title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.title, 800, 480)
        cv2.imshow(self.title, image)
        self.created = True
        key = cv2.waitKey(1) & 0xFF
        return key not in (27, ord("q")) and self.visible()

    def close(self):
        if self.created:
            try:
                cv2.destroyWindow(self.title)
                cv2.waitKey(1)
            except cv2.error:
                pass
        self.created = False
