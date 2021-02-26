from pathlib import Path
from py_md_doc import PyMdDoc


if __name__ == "__main__":
    md = PyMdDoc(input_directory=Path("../transport_challenge"), files=["transport_controller.py"],
                 metadata_path=Path("doc_metadata.json"))
    md.get_docs(output_directory=Path("../doc"))
