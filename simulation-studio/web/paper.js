'use strict';
const el = id => document.getElementById(id);
let language = 'zh', paper, paperId, page = 1, focus = '';
try {
  language = localStorage.getItem('simulation-studio.language') === 'en' ? 'en' : 'zh';
  document.documentElement.dataset.theme = localStorage.getItem('simulation-studio.theme') || 'light';
} catch { /* The offline reader also works without browser storage. */ }
const tr = (zh, en) => language === 'en' ? en : zh;
document.documentElement.lang = language === 'en' ? 'en' : 'zh-CN';

function renderPage(next, location = '') {
  page = Math.min(paper.pages, Math.max(1, Number.isFinite(next) ? Math.trunc(next) : 1));
  focus = location;
  const bookmark = paper.locations[focus];
  const active = bookmark && bookmark.page === page;
  el('pageNumber').value = page;
  el('pageNumber').max = paper.pages;
  el('pageCount').textContent = `/ ${paper.pages}`;
  el('previous').disabled = page <= 1;
  el('next').disabled = page >= paper.pages;
  el('location').textContent = active ? bookmark.label[language === 'en' ? 1 : 0] : tr(`PDF 第 ${page} 页`, `PDF page ${page}`);
  el('hint').textContent = tr('显示下载原文的完整页面。黄色标记是引用关键词；PDF 页码可能与刊物印刷页码不同。', 'This is a full page rendered from the downloaded original. Yellow marks locate cited keywords; PDF and printed page numbers may differ.');
  el('highlights').replaceChildren();
  el('pageImage').alt = `${paper.title} · ${tr('PDF 第', 'PDF page')} ${page}`;
  el('pageImage').src = `/literature/${paperId}/page-${String(page).padStart(3, '0')}.png`;
  el('originalPdf').href = `${paper.local_pdf}#page=${page}`;
  el('error').hidden = true;
  for (const box of active ? (paper.highlights[focus] || []) : []) {
    const mark = document.createElement('span');
    mark.className = 'word-highlight';
    Object.assign(mark.style, {left:`${box.x*100}%`, top:`${box.y*100}%`, width:`${box.w*100}%`, height:`${box.h*100}%`});
    el('highlights').append(mark);
  }
  el('locateCitation').disabled = !el('highlights').firstElementChild;
  const url = new URL(locationHref());
  history.replaceState(null, '', url);
}
function locationHref() {
  const url = new URL(window.location.href);
  url.searchParams.set('paper', paperId);
  url.searchParams.set('page', String(page));
  if (focus) url.searchParams.set('focus', focus); else url.searchParams.delete('focus');
  return url.href;
}
function locateCitation() {
  el('highlights').firstElementChild?.scrollIntoView({block:'center', inline:'center'});
}
async function init() {
  const params = new URLSearchParams(window.location.search);
  paperId = params.get('paper');
  const response = await fetch('/api/literature');
  if (!response.ok) throw Error(tr('无法读取参考资料目录。', 'Could not load the literature catalog.'));
  const catalog = await response.json();
  paper = catalog[paperId];
  if (!paper) throw Error(tr('未找到这篇参考资料。', 'Unknown paper.'));
  document.title = paper.title;
  el('title').textContent = paper.title;
  el('byline').textContent = `${paper.authors} · ${paper.publication}`;
  el('eyebrow').textContent = tr('SIMULATION STUDIO / 本地参考资料', 'SIMULATION STUDIO / LOCAL REFERENCES');
  el('originalPdf').textContent = tr('打开完整 PDF ↗', 'Open full PDF ↗');
  el('downloadPdf').textContent = tr('下载 PDF', 'Download PDF');
  el('publisher').textContent = tr('出版来源 ↗', 'Publication source ↗');
  el('publisher').href = paper.doi || paper.url;
  el('downloadPdf').href = paper.local_pdf;
  el('downloadPdf').download = `${paperId}.pdf`;
  el('downloads').hidden = false;
  if (!paper.available || !paper.rendered) throw Error(tr('论文页面尚未准备完成，可先打开出版来源。', 'Local pages are not ready; use the publication source.'));
  el('previous').textContent = tr('← 上一页', '← Previous');
  el('next').textContent = tr('下一页 →', 'Next →');
  el('locateCitation').textContent = tr('定位引用', 'Locate citation');
  el('pageLabel').textContent = tr('PDF 页码', 'PDF page');
  el('goPage').textContent = tr('跳转', 'Go');
  el('zoomLabel').textContent = tr('缩放', 'Zoom');
  el('zoom').options[0].textContent = tr('适应窗口', 'Fit window');
  for (const [key, bookmark] of Object.entries(paper.locations)) {
    const link = document.createElement('a');
    link.textContent = bookmark.label[language === 'en' ? 1 : 0];
    link.href = `/paper?paper=${paperId}&page=${bookmark.page}&focus=${key}`;
    link.onclick = event => {event.preventDefault(); renderPage(bookmark.page, key);};
    el('bookmarks').append(link);
  }
  el('readerControls').hidden = false;
  el('viewport').hidden = false;
  focus = params.get('focus') || '';
  const initialPage = Number(params.get('page') || paper.locations[focus]?.page || 1);
  renderPage(initialPage, focus);
}
el('pageForm').onsubmit = event => {event.preventDefault(); renderPage(Number(el('pageNumber').value));};
el('previous').onclick = () => renderPage(page - 1);
el('next').onclick = () => renderPage(page + 1);
el('locateCitation').onclick = locateCitation;
el('pageImage').onload = () => {if (focus) locateCitation();};
el('zoom').onchange = () => {
  el('pageWrap').style.width = el('zoom').value === 'fit' ? 'min(100%,1050px)' : `${10.5*Number(el('zoom').value)}px`;
};
el('pageImage').onerror = () => {el('error').textContent = tr('页面图片读取失败，请打开完整 PDF。', 'Page image could not load; open the full PDF.'); el('error').hidden = false;};
init().catch(error => {el('error').textContent = error.message; el('error').hidden = false;});
