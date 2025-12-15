/**
 * WIA Cognitive AAC - Adaptive Grid Component
 * 인지 프로파일 기반 적응형 그리드 컴포넌트
 *
 * 홍익인간 - 널리 인간을 이롭게 하라
 */

import React, { useMemo, useCallback, useState } from 'react';
import {
  SymbolItem,
  SymbolPage,
  UIConfiguration,
  GridConfig,
  CognitiveLoadConfig,
} from '../types';
import { CELL_SIZE_PX } from '../engine/Presets';
import { AdaptiveSymbol } from './AdaptiveSymbol';

// ============================================================================
// Types
// ============================================================================

export interface AdaptiveGridProps {
  page: SymbolPage;
  config: UIConfiguration;
  selectedItems?: string[];
  highlightedItem?: string | null;
  onSelectItem?: (item: SymbolItem) => void;
  onNavigate?: (pageId: string) => void;
  onHoverItem?: (item: SymbolItem | null) => void;
  className?: string;
  style?: React.CSSProperties;
}

interface GridDimensions {
  columns: number;
  rows: number;
  totalCells: number;
  cellWidth: number;
  cellHeight: number;
  gap: number;
}

// ============================================================================
// Component
// ============================================================================

export const AdaptiveGrid: React.FC<AdaptiveGridProps> = ({
  page,
  config,
  selectedItems = [],
  highlightedItem = null,
  onSelectItem,
  onNavigate,
  onHoverItem,
  className = '',
  style = {},
}) => {
  const [currentPage, setCurrentPage] = useState(0);

  // 그리드 차원 계산
  const dimensions = useMemo((): GridDimensions => {
    const gridConfig = page.layout
      ? { ...config.grid, ...page.layout }
      : config.grid;

    return {
      columns: gridConfig.columns,
      rows: gridConfig.rows,
      totalCells: gridConfig.columns * gridConfig.rows,
      cellWidth: CELL_SIZE_PX[gridConfig.cellSize],
      cellHeight: CELL_SIZE_PX[gridConfig.cellSize],
      gap: gridConfig.gap || 8,
    };
  }, [config.grid, page.layout]);

  // 인지 부하 기반 심볼 필터링
  const visibleSymbols = useMemo(() => {
    const maxItems = config.cognitiveLoad.maxVisibleItems;
    const filtered = page.symbols.slice(0, maxItems);

    // Progressive disclosure가 활성화된 경우 페이지네이션
    if (config.cognitiveLoad.progressiveDisclosure) {
      const itemsPerPage = dimensions.totalCells;
      const startIndex = currentPage * itemsPerPage;
      return filtered.slice(startIndex, startIndex + itemsPerPage);
    }

    return filtered.slice(0, dimensions.totalCells);
  }, [page.symbols, config.cognitiveLoad, dimensions.totalCells, currentPage]);

  // 총 페이지 수 계산
  const totalPages = useMemo(() => {
    if (!config.cognitiveLoad.progressiveDisclosure) return 1;

    const maxItems = Math.min(
      config.cognitiveLoad.maxVisibleItems,
      page.symbols.length
    );
    return Math.ceil(maxItems / dimensions.totalCells);
  }, [config.cognitiveLoad, page.symbols.length, dimensions.totalCells]);

  // 아이템 선택 핸들러
  const handleSelectItem = useCallback(
    (item: SymbolItem) => {
      if (item.action === 'navigate' && item.targetPageId) {
        onNavigate?.(item.targetPageId);
      } else {
        onSelectItem?.(item);
      }
    },
    [onSelectItem, onNavigate]
  );

  // Dwell 완료 핸들러
  const handleDwellComplete = useCallback(
    (item: SymbolItem) => {
      handleSelectItem(item);
    },
    [handleSelectItem]
  );

  // 페이지 이동 핸들러
  const goToPage = useCallback((pageNum: number) => {
    setCurrentPage(Math.max(0, Math.min(pageNum, totalPages - 1)));
  }, [totalPages]);

  // 그리드 컨테이너 스타일
  const containerStyle: React.CSSProperties = {
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
    gap: '16px',
    padding: '16px',
    ...style,
  };

  // 그리드 스타일
  const gridStyle: React.CSSProperties = {
    display: 'grid',
    gridTemplateColumns: `repeat(${dimensions.columns}, ${dimensions.cellWidth}px)`,
    gridTemplateRows: `repeat(${dimensions.rows}, ${dimensions.cellHeight}px)`,
    gap: `${dimensions.gap}px`,
    justifyContent: 'center',
    alignContent: 'center',
  };

  // 페이지네이션 스타일
  const paginationStyle: React.CSSProperties = {
    display: 'flex',
    gap: '8px',
    alignItems: 'center',
    justifyContent: 'center',
  };

  const pageButtonStyle = (isActive: boolean): React.CSSProperties => ({
    width: '40px',
    height: '40px',
    borderRadius: '50%',
    border: 'none',
    backgroundColor: isActive ? '#007AFF' : '#E0E0E0',
    color: isActive ? '#FFFFFF' : '#333333',
    fontSize: '16px',
    fontWeight: 600,
    cursor: 'pointer',
    transition: 'all 0.2s ease',
  });

  const navButtonStyle: React.CSSProperties = {
    width: '48px',
    height: '48px',
    borderRadius: '8px',
    border: 'none',
    backgroundColor: '#F0F0F0',
    color: '#333333',
    fontSize: '20px',
    cursor: 'pointer',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
  };

  return (
    <div className={`adaptive-grid ${className}`} style={containerStyle}>
      {/* 페이지 타이틀 */}
      {page.title && (
        <h2 style={{
          margin: 0,
          fontSize: config.symbols.fontSize * 1.5,
          fontWeight: 600,
          color: '#333333',
        }}>
          {page.title}
        </h2>
      )}

      {/* 심볼 그리드 */}
      <div style={gridStyle} role="grid" aria-label={page.title || 'Symbol grid'}>
        {visibleSymbols.map((item, index) => (
          <div
            key={item.id}
            role="gridcell"
            aria-rowindex={Math.floor(index / dimensions.columns) + 1}
            aria-colindex={(index % dimensions.columns) + 1}
          >
            <AdaptiveSymbol
              item={item}
              symbolConfig={config.symbols}
              interactionConfig={config.interaction}
              cellSize={config.grid.cellSize}
              isSelected={selectedItems.includes(item.id)}
              isHighlighted={highlightedItem === item.id}
              onSelect={handleSelectItem}
              onHover={onHoverItem}
              onDwellComplete={handleDwellComplete}
            />
          </div>
        ))}

        {/* 빈 셀 채우기 */}
        {Array.from({ length: dimensions.totalCells - visibleSymbols.length }).map(
          (_, index) => (
            <div
              key={`empty-${index}`}
              style={{
                width: dimensions.cellWidth,
                height: dimensions.cellHeight,
                backgroundColor: 'transparent',
              }}
            />
          )
        )}
      </div>

      {/* 페이지네이션 (progressive disclosure 활성화 시) */}
      {config.cognitiveLoad.progressiveDisclosure && totalPages > 1 && (
        <nav style={paginationStyle} aria-label="Page navigation">
          <button
            style={navButtonStyle}
            onClick={() => goToPage(currentPage - 1)}
            disabled={currentPage === 0}
            aria-label="Previous page"
          >
            &#8249;
          </button>

          {Array.from({ length: totalPages }).map((_, index) => (
            <button
              key={index}
              style={pageButtonStyle(index === currentPage)}
              onClick={() => goToPage(index)}
              aria-label={`Page ${index + 1}`}
              aria-current={index === currentPage ? 'page' : undefined}
            >
              {index + 1}
            </button>
          ))}

          <button
            style={navButtonStyle}
            onClick={() => goToPage(currentPage + 1)}
            disabled={currentPage === totalPages - 1}
            aria-label="Next page"
          >
            &#8250;
          </button>
        </nav>
      )}
    </div>
  );
};

export default AdaptiveGrid;
