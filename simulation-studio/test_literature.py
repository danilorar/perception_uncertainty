"""Local original papers, page locations, range serving and path boundaries."""
import hashlib
import json
import threading
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from app import Handler, ThreadingHTTPServer
from literature import PAPERS, paper_catalog, paper_file


class LiteratureTests(unittest.TestCase):
    def test_downloads_and_page_locators_match_verified_manifest(self):
        for ident, paper in paper_catalog().items():
            with self.subTest(paper=ident):
                self.assertTrue(paper['available'])
                self.assertTrue(paper['rendered'])
                self.assertEqual(hashlib.sha256(paper_file(ident,'source.pdf').read_bytes()).hexdigest(),paper['sha256'])
                for location, details in paper['locations'].items():
                    self.assertTrue(paper_file(ident,f"page-{details['page']:03}.png").is_file())
                    self.assertTrue(paper['highlights'][location])
                    for box in paper['highlights'][location]:
                        self.assertTrue(all(0 <= v <= 1 for v in box.values()))
                        self.assertLessEqual(box['x']+box['w'],1.001)
                        self.assertLessEqual(box['y']+box['h'],1.001)

    def test_only_registered_files_are_accessible(self):
        for ident, filename in [('unknown','source.pdf'),('yang-2020','../../models.py'),
                                ('yang-2020','page-999.png'),('../','source.pdf')]:
            with self.assertRaises(ValueError): paper_file(ident,filename)

    def test_http_reader_pdf_range_and_page_images(self):
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        thread=threading.Thread(target=server.serve_forever,daemon=True)
        thread.start()
        base=f'http://127.0.0.1:{server.server_port}'
        try:
            with urlopen(base+'/api/literature') as response:
                self.assertEqual(set(json.load(response)),set(PAPERS))
            with urlopen(base+'/paper?paper=yang-2020&page=3&focus=transition') as response:
                self.assertIn(b'pageImage',response.read())
            request=Request(base+'/literature/yang-2020/source.pdf',headers={'Range':'bytes=0-31'})
            with urlopen(request) as response:
                self.assertEqual(response.status,206)
                self.assertEqual(response.headers['Content-Type'],'application/pdf')
                self.assertEqual(response.read(),paper_file('yang-2020','source.pdf').read_bytes()[:32])
            with urlopen(base+'/literature/turlej-2023/page-006.png') as response:
                self.assertTrue(response.read(8).startswith(b'\x89PNG'))
            with self.assertRaises(HTTPError): urlopen(base+'/literature/yang-2020/models.py')
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


if __name__ == '__main__': unittest.main(verbosity=2)
