import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from ArgosConfig.ToArgos import create_Argos


class VisualizerModesTest(unittest.TestCase):
    def render(self, visualizer):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "test.argos"
            create_Argos(
                map_data=["..", ".."],
                output_file_path=str(output),
                width=2,
                height=2,
                robot_init_pos=[("0", "0")],
                curr_num_agent=1,
                port_num=8182,
                n_threads=1,
                visualizer=visualizer,
            )
            return ET.parse(output).getroot()

    def test_modes_select_the_expected_argos_visualizer(self):
        none = self.render("none")
        self.assertIsNone(none.find("visualization"))
        self.assertEqual(
            none.find("./framework/experiment").get("visualization"), "none"
        )

        web = self.render("web")
        self.assertIsNotNone(web.find("./visualization/external_visualizer"))
        self.assertIsNone(web.find("./visualization/qt-opengl"))

        argos = self.render("argos")
        self.assertIsNotNone(argos.find("./visualization/qt-opengl"))
        self.assertIsNone(argos.find("./visualization/external_visualizer"))

    def test_invalid_mode_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "none, web, argos"):
            self.render("invalid")


if __name__ == "__main__":
    unittest.main()
