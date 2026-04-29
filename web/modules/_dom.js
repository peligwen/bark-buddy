// web/modules/_dom.js
//
// Tiny DOM-builder helpers. Forty lines, no framework. Lifted from diag.js
// where they were the only thing worth re-using when a second debug panel
// (SLAM, mapping) eventually arrives.
//
//   el('div', {class: 'foo'}, ['hello', el('b', null, ['world'])])
//   row([label('X'), input({type: 'number'}), btn('Go', onGo)])
//
// `attrs.class` maps to className; `attrs.style` to cssText; everything else
// becomes a setAttribute call. Children accept strings (text nodes) or DOM
// nodes; null/undefined children are skipped.

export function el(tag, attrs, children) {
    const e = document.createElement(tag);
    if (attrs) {
        for (const k of Object.keys(attrs)) {
            const v = attrs[k];
            if (v == null) continue;
            if (k === 'class')      e.className = v;
            else if (k === 'style') e.style.cssText = v;
            else                     e.setAttribute(k, v);
        }
    }
    if (children) {
        for (const c of children) {
            if (c == null) continue;
            if (typeof c === 'string') e.appendChild(document.createTextNode(c));
            else                        e.appendChild(c);
        }
    }
    return e;
}

export function input(attrs) { return el('input', attrs); }

export function label(text, forId) {
    const l = document.createElement('label');
    l.textContent = text;
    if (forId) l.setAttribute('for', forId);
    return l;
}

export function btn(text, onClick, opts) {
    const b = el('button', { class: (opts && opts.class) || 'diag-btn' }, [text]);
    if (onClick) b.addEventListener('click', onClick);
    return b;
}

export function row(children, opts) {
    return el('div', { class: (opts && opts.class) || 'diag-row' }, children);
}

export function section(title, children, opts) {
    const cls = (opts && opts.class) || 'diag-section';
    const titleCls = (opts && opts.titleClass) || 'diag-section-title';
    return el('div', { class: cls }, [el('h3', { class: titleCls }, [title])].concat(children));
}
