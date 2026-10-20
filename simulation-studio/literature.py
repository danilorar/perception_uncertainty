"""Allowlisted, local research papers and precise reading locations."""
import json
from pathlib import Path

from settings import ROOT

PAPER_ROOT = ROOT / 'assets/literature'
PAPERS = {
    'turlej-2023': {
        'title': 'High-Level Sensor Models for the Reinforcement Learning Driving Policy Training',
        'authors': 'Wojciech Turlej', 'year': 2023, 'pages': 20,
        'publication': 'Electronics 12(1), 71 (online 25 Dec 2022)',
        'url': 'https://mdpi-res.com/d_attachment/electronics/electronics-12-00071/article_deploy/electronics-12-00071.pdf',
        'doi': 'https://doi.org/10.3390/electronics12010071',
        'locations': {
            'temporal': {'page': 6, 'keywords': ['Ornstein', 'time-correlated'],
                         'label': ['第 6 页 · 时间相关状态误差', 'p. 6 · Time-correlated state errors']},
        },
    },
    'yang-2020': {
        'title': 'Bias Estimation for Collocated Sensors: Model Identification and Measurement Fusion',
        'authors': 'Kaipei Yang, Yaakov Bar-Shalom, Peter Willett, Hiroshi Inou',
        'year': 2020, 'pages': 10,
        'publication': 'Journal of Advances in Information Fusion 15(2), 91–100',
        'url': 'https://isif.org/files/isif/2024-01/Bias%20Estimation.pdf',
        'doi': '',
        'locations': {
            'transition': {'page': 3, 'keywords': ['Gauss', 'discrete', 'variances'],
                           'label': ['PDF 第 3 页 / 刊物第 93 页 · 公式 (3)、(4)、(7)',
                                     'PDF p. 3 / journal p. 93 · Equations (3), (4), (7)']},
            'calibration': {'page': 4, 'keywords': ['Autocorrelation', 'autocorrelation'],
                            'label': ['PDF 第 4 页 / 刊物第 94 页 · 自相关与参数辨识',
                                      'PDF p. 4 / journal p. 94 · Autocorrelation and identification']},
        },
    },
    'piazzoni-2024': {
        'title': 'PEM: Perception Error Model for Virtual Testing of Autonomous Vehicles',
        'authors': 'Andrea Piazzoni, Jim Cherian, Justin Dauwels, Lap-Pui Chau',
        'year': 2024, 'pages': 14,
        'publication': 'IEEE T-ITS 25(1), 670–681 (online 2023)',
        'url': 'https://pure.tudelft.nl/ws/portalfiles/portal/176408023/PEM_Perception_Error_Model_for_Virtual_Testing_of_Autonomous_Vehicles.pdf',
        'doi': 'https://doi.org/10.1109/TITS.2023.3311633',
        'locations': {
            'temporal': {'page': 7, 'keywords': ['Temporal', 'Validation'],
                         'label': ['PDF 第 7 页 / 刊物第 674 页 · IV-B.4 时间因素与验证',
                                   'PDF p. 7 / journal p. 674 · IV-B.4 Temporal aspects and validation']},
        },
    },
    'sensi-2025': {
        'title': 'Collaborative Approach to Sensor Perception Modelling (SENSI)',
        'authors': 'Amin Assadi, Jordanka Kovaceva et al.', 'year': 2025, 'pages': 1,
        'publication': 'Chalmers research poster / preprint',
        'url': 'https://research.chalmers.se/publication/549591/file/549591_Fulltext.pdf',
        'doi': '',
        'locations': {
            'consistency': {'page': 1, 'keywords': ['temporal', 'consistency'],
                            'label': ['第 1 页 · temporal consistency error',
                                      'p. 1 · Temporal consistency error']},
        },
    },
}


def paper_file(paper_id, filename):
    if paper_id not in PAPERS:
        raise ValueError('未知参考资料')
    allowed = {'source.pdf', 'manifest.json'} | {
        f'page-{page:03}.png' for page in range(1, PAPERS[paper_id]['pages'] + 1)
    }
    if filename not in allowed:
        raise ValueError('无效参考资料路径')
    root = PAPER_ROOT.resolve()
    path = (PAPER_ROOT / paper_id / filename).resolve()
    if not path.is_relative_to(root):
        raise ValueError('参考资料路径超出项目目录')
    return path


def paper_catalog():
    result = {}
    for paper_id, info in PAPERS.items():
        entry = dict(info)
        entry['id'] = paper_id
        entry['local_pdf'] = f'/literature/{paper_id}/source.pdf'
        entry['available'] = paper_file(paper_id, 'source.pdf').is_file()
        manifest_path = paper_file(paper_id, 'manifest.json')
        try:
            manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            manifest = {}
        entry['rendered'] = bool(manifest.get('complete'))
        entry['sha256'] = manifest.get('sha256')
        entry['highlights'] = manifest.get('highlights', {})
        result[paper_id] = entry
    return result
