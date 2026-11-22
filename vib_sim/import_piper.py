"""PiPER 공식 URDF → USD (질량·관성을 URDF 값으로 쓰기 위함). 결과: assets/piper_urdf/"""
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import os
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
here = os.path.dirname(os.path.abspath(__file__))
cfg = URDFImporterConfig(urdf_path=here + "/assets/piper_abs.urdf", usd_path=here + "/assets",
                         fix_base=False, collision_from_visuals=False)
print("IMPORT_DONE:", URDFImporter(cfg).import_urdf())
app.close()
